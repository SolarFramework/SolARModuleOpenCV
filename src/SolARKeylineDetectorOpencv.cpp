/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "SolARKeylineDetectorOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARKeylineDetectorOpencv)

namespace xpcf = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

static std::map<std::string, api::features::KeylineDetectorType> stringToType = {
	{ "HOUGH", api::features::KeylineDetectorType::HOUGH },
};

static std::map<api::features::KeylineDetectorType, std::string> typeToString = {
	{ api::features::KeylineDetectorType::HOUGH, "HOUGH" },
};

SolARKeylineDetectorOpencv::SolARKeylineDetectorOpencv() : ConfigurableBase(xpcf::toUUID<SolARKeylineDetectorOpencv>())
{
	declareInterface<api::features::IKeylineDetector>(this);

	declareProperty("imageRatio", m_imageRatio);
	declareProperty("reductionRatio", m_reductionRatio);
	declareProperty("numOctaves", m_numOctaves);
	declareProperty("type", m_type);
	declareProperty("minLineLength", m_minLineLength);
    declareProperty("maxLineGap", m_maxLineGap);
    declareProperty("distanceResolution", m_distanceResolution);
	declareProperty("angleResolution", m_angleResolution);
	declareProperty("threshold", m_threshold);
	LOG_DEBUG("SolARKeylineDetectorOpencv constructor");
}

SolARKeylineDetectorOpencv::~SolARKeylineDetectorOpencv()
{
    LOG_DEBUG("SolARKeylineDetectorOpencv destructor");
}

xpcf::XPCFErrorCode SolARKeylineDetectorOpencv::onConfigured()
{
    LOG_DEBUG("SolARKeylineDetectorOpencv onConfigured");
    if (stringToType.find(m_type) != stringToType.end())
	{
		return initDetector();
	}
	else
	{
		LOG_WARNING("Keyline detector of type {} defined in your configuration file is not supported", m_type);
		return xpcf::_ERROR_NOT_IMPLEMENTED;
	}
}

xpcf::XPCFErrorCode SolARKeylineDetectorOpencv::setType(api::features::KeylineDetectorType type)
{
    if (typeToString.find(type) != typeToString.end())
	{
		m_type = typeToString.at(type);
		return initDetector();
	}
	else
	{
		LOG_WARNING("Unsupported KeylineDetectorType: falling back to previous type {}", m_type);
		return xpcf::_ERROR_TYPE;
	}
}

xpcf::XPCFErrorCode SolARKeylineDetectorOpencv::initDetector()
{
    return xpcf::_SUCCESS;
}

api::features::KeylineDetectorType SolARKeylineDetectorOpencv::getType()
{
	return stringToType.at(m_type);
}

void SolARKeylineDetectorOpencv::detect(const SRef<datastructure::Image> image, std::vector<datastructure::Keyline>& keylines)
{
    float ratioInv = 1.f / m_imageRatio;

	cv::Mat opencvImage = SolAROpenCVHelper::mapToOpenCV(image);
	cv::resize(opencvImage, opencvImage, cv::Size(), m_imageRatio, m_imageRatio);
	// Conversion to grayscale
	if (opencvImage.channels() != 1)
		cv::cvtColor(opencvImage, opencvImage, cv::COLOR_BGR2GRAY);
	try
    {
        switch( getType() )
        {
        case api::features::KeylineDetectorType::HOUGH:
        {
            // Prepare different scale/octave
			std::vector<std::vector<cv::Vec4f>> lines;
			lines.resize(m_numOctaves);
			std::vector<cv::Mat> gaussianPyrs = computeGaussianPyramids(opencvImage, m_numOctaves, m_reductionRatio);
			// Perform line detection on each octave
			int lineCount{0};
            for (int i = 0; i < m_numOctaves; ++i)
            {
				// Edge detection filter
				cv::Canny(gaussianPyrs[i], gaussianPyrs[i], 50, 200, 3);
				// Line extraction
                cv::HoughLinesP(gaussianPyrs[i], lines[i],
                    m_distanceResolution, m_angleResolution, m_threshold, m_minLineLength, m_maxLineGap);
                lineCount += lines[i].size();
            }
            // Create keylines
			keylines.resize(lineCount);
			int classCounter{0};
			float octaveScale = 1.f;
			for (int i = 0; i < m_numOctaves; ++i)
			{
				for (const auto& l : lines[i])
				{
					// Fill keyline's fields
					auto startPointInOctave = cv::Point2f(l[0], l[1]);
					auto endPointInOctave   = cv::Point2f(l[2], l[3]);
					float startPointX = startPointInOctave.x * octaveScale;
					float startPointY = startPointInOctave.y * octaveScale;
					float endPointX = endPointInOctave.x * octaveScale;
					float endPointY = endPointInOctave.y * octaveScale;
					float lineLength = std::sqrt(
						std::pow(startPointInOctave.x - endPointInOctave.x, 2.f) +
						std::pow(startPointInOctave.y - endPointInOctave.y, 2.f)
					);

					// Compute number of pixels covered by line
					cv::LineIterator li(gaussianPyrs[i], startPointInOctave, endPointInOctave);
					int numOfPixels = li.count;

					float dx = endPointX - startPointX, dy = endPointY - startPointY;
					float angle = std::atan2(dy, dx);
					// Ideally, keylines representing the same line (ie. accross octaves) should have the same id
					// That processing is skipped here, we only make sure each line has a unique id
					int classId = classCounter++;
					int octave = i;
					float size = dx * dy;
					float response = lineLength / std::max(gaussianPyrs[i].cols, gaussianPyrs[i].rows);
					auto midPoint = cv::Point2f((endPointX + startPointX) / 2.f, (endPointY + startPointY) / 2.f);

					keylines[classId] = Keyline(
						midPoint.x * ratioInv,
						midPoint.y * ratioInv,
						startPointX * ratioInv,
						startPointY * ratioInv,
						startPointInOctave.x * ratioInv,
						startPointInOctave.y * ratioInv,
						endPointX * ratioInv,
						endPointY * ratioInv,
						endPointInOctave.x * ratioInv,
						endPointInOctave.y * ratioInv,
						lineLength * ratioInv,
						size * ratioInv,
						angle,
						response,
						numOfPixels * ratioInv,
						octave,
						classId
					);
				}
				// Prepare next octave scale factor
				if (i < m_numOctaves - 1)
					octaveScale *= m_reductionRatio;
            }
            break;
        }
        }        
    }
    catch(const cv::Exception & e)
    {
		LOG_ERROR("{}", e.msg);
		return;
    }        
}

std::vector<cv::Mat> SolARKeylineDetectorOpencv::computeGaussianPyramids(const cv::Mat & opencvImage, int numOctaves, int scale)
{
	std::vector<cv::Mat> gaussianPyrs;
	gaussianPyrs.resize(numOctaves);
	gaussianPyrs[0] = opencvImage.clone();
	int size_x = gaussianPyrs[0].cols, size_y = gaussianPyrs[0].rows;
	for (int i = 1; i < numOctaves; ++i)
	{
		cv::pyrDown( gaussianPyrs[i - 1], gaussianPyrs[i], cv::Size(size_x /= scale, size_y /= scale) );
	}
	return gaussianPyrs;
}

}
}
}
