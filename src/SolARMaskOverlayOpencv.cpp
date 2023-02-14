/**
 * @copyright Copyright (c) 2020 B-com http://www.b-com.com/
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

#include "SolARMaskOverlayOpencv.h"
#include "core/Log.h"
#include "SolAROpenCVHelper.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARMaskOverlayOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARMaskOverlayOpencv::SolARMaskOverlayOpencv():ConfigurableBase(xpcf::toUUID<SolARMaskOverlayOpencv>())
{
    LOG_DEBUG("SolARMaskOverlayOpencv constructor")
    declareInterface<api::display::IMaskOverlay>(this);
    declareProperty("classFile", m_classFile);
    declareProperty("colorFile", m_colorFile);
	declarePropertySequence("otherClassColor", m_otherClassColor);
}

SolARMaskOverlayOpencv::~SolARMaskOverlayOpencv()
{
    LOG_DEBUG(" SolARMaskOverlayOpencv destructor")
}

xpcf::XPCFErrorCode SolARMaskOverlayOpencv::onConfigured()
{
    LOG_DEBUG(" SolARMaskOverlayOpencv onConfigured");
	// Load names of classes
	std::ifstream ifs(m_classFile.c_str());
	std::string line;	
	while (std::getline(ifs, line)) 
        m_classes.push_back(line);

    // Load the colors	
    std::ifstream colorFptr(m_colorFile.c_str());
    while (std::getline(colorFptr, line)) {
        std::istringstream iss(line);
        double r, g, b;
        iss >> r >> g >> b;
        m_colors.push_back(cv::Scalar(b, g, r, 255.0));
    }
	
    // extract effective class and colors for legend display 
    for (int i = 0; i < static_cast<int>(m_colors.size()); i++) {
        if ((static_cast<int>(m_colors[i][2]) != m_otherClassColor[0]) ||
            (static_cast<int>(m_colors[i][1]) != m_otherClassColor[1]) ||
            (static_cast<int>(m_colors[i][0]) != m_otherClassColor[2])) {
            m_classes_legend.push_back(m_classes[i]);
            m_colors_legend.push_back(m_colors[i]);
        }
    }
    m_classes_legend.push_back("Other");
    m_colors_legend.push_back(cv::Scalar(m_otherClassColor[2], m_otherClassColor[1], m_otherClassColor[0], 255.0));

    return xpcf::XPCFErrorCode::_SUCCESS;
}

FrameworkReturnCode SolARMaskOverlayOpencv::draw(SRef<SolAR::datastructure::Image> image,
                                                 const std::vector<SolAR::datastructure::Rectanglei> &boxes,
                                                 const std::vector<SRef<SolAR::datastructure::Image>> &masks,
                                                 const std::vector<uint32_t> &classIds,
                                                 const std::vector<float> &scores)
{
	// convert to opencv image
	cv::Mat imageCV = SolAROpenCVHelper::mapToOpenCV(image);
	if (imageCV.channels() != 3) {
		LOG_ERROR("Input image must be RGB image");
		return FrameworkReturnCode::_ERROR_;
	}

	// Draw
	int nbBoxes = boxes.size();
	for (int i = 0; i < nbBoxes; ++i) {
		cv::Rect box(boxes[i].startX, boxes[i].startY, boxes[i].size.width, boxes[i].size.height);
		cv::Scalar color = m_colors[classIds[i] % m_colors.size()];
		//Draw a rectangle displaying the bounding box
		cv::rectangle(imageCV, box, color, 3);

		//Get the label for the class name and its confidence
		std::string label = cv::format("%.2f", scores[i]);
		CV_Assert(classIds[i] < m_classes.size());
		label = m_classes[classIds[i]] + ":" + label;

		//Display the label at the top of the bounding box
		int baseLine;
		cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
		int boxY = std::max(box.y, labelSize.height);
		cv::rectangle(imageCV, cv::Point(box.x, boxY - round(1.5 * labelSize.height)), cv::Point(box.x + round(1.5*labelSize.width), boxY + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
		cv::putText(imageCV, label, cv::Point(box.x, boxY), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);

		cv::Mat coloredRoi = (0.3 * color + 0.7 * imageCV(box));
		coloredRoi.convertTo(coloredRoi, CV_8UC3);
		// Draw the contours on the image
		std::vector<cv::Mat> contours;
		cv::Mat hierarchy;
		cv::Mat mask = SolAROpenCVHelper::mapToOpenCV(masks[i]);
		mask.convertTo(mask, CV_8U);
		cv::findContours(mask, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
		cv::drawContours(coloredRoi, contours, -1, color, 5, cv::LINE_8, hierarchy, 100);
		coloredRoi.copyTo(imageCV(box), mask);
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMaskOverlayOpencv::draw(SRef<SolAR::datastructure::Image> image, 
												 const SRef<SolAR::datastructure::Image> mask)
{
	// convert to opencv image
	cv::Mat imageCV = SolAROpenCVHelper::mapToOpenCV(image);
	if (imageCV.channels() != 3) {
		LOG_ERROR("Input image must be RGB image");
		return FrameworkReturnCode::_ERROR_;
	}
	int rows = imageCV.rows;
	int cols = imageCV.cols;
	cv::Mat maskCV = SolAROpenCVHelper::mapToOpenCV(mask);
	cv::Mat overlay(rows, cols, CV_8UC3);
	for (int row = 0; row < rows; row++)
	{
		const uchar *ptrMaskCV = maskCV.ptr<uchar>(row);
		cv::Vec3b *ptrOverlay = overlay.ptr<cv::Vec3b>(row);
		for (int col = 0; col < cols; col++)
		{
			uchar classId = ptrMaskCV[col];
			if (classId >= m_colors.size()) {
				LOG_ERROR("Class id exceeds the number of objects");
				return FrameworkReturnCode::_ERROR_;
			}
			ptrOverlay[col] = cv::Vec3b(m_colors[classId][0], m_colors[classId][1], m_colors[classId][2]);
		}
	}
	cv::addWeighted(imageCV, 0.3, overlay, 0.7, 0.0, imageCV);
	// show legend
	static const int numClassesPerLegend = 30;
	static const int kBlockHeight = rows / numClassesPerLegend;
	static cv::Mat legend, legend2;

    auto fnGenLegend = [&](auto& lgd, int nclasses, int offset) {
        lgd.create(rows, 100, CV_8UC3);
        lgd.setTo(cv::Scalar(0,0,0));
        for (int i = 0; i < nclasses; i++) {
            cv::Mat block = lgd.rowRange(i * kBlockHeight, (i + 1) * kBlockHeight);
            block.setTo(cv::Vec3b(m_colors_legend[i+ offset][0], m_colors_legend[i+ offset][1], m_colors_legend[i+ offset][2]));
            putText(block, m_classes_legend[i+ offset], cv::Point(0, kBlockHeight / 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Vec3b(0, 0, 0));
        }
    };

    if (legend.empty())
    {
        const int numClasses = std::min<int>(numClassesPerLegend,(int)m_classes_legend.size());
        fnGenLegend(legend, numClasses, 0);
    }
    legend.copyTo(imageCV(cv::Rect(cols - 100, 0, 100, rows)));
    // number of classes between 31 and 60, need a second legend
    if (static_cast<int>(m_classes_legend.size()) > numClassesPerLegend) {
        if (legend2.empty()) {
            const int numClasses2 = std::min<int>(numClassesPerLegend, (int)m_classes_legend.size() - numClassesPerLegend);
            fnGenLegend(legend2, numClasses2, numClassesPerLegend);
        }
        legend2.copyTo(imageCV(cv::Rect(0, 0, 100, rows)));
    }
   
    if (static_cast<int>(m_classes_legend.size()) > 2*numClassesPerLegend) {
        LOG_WARNING("Number of classes to display is greater than current limit {}", 2*numClassesPerLegend);
        // TODO: if more than 60 classes, need to handle it properly
    }
    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
