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

#include "SolARCornerRefinementOpencv.h"
#include "SolARImageConvertorOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include <opencv2/imgproc.hpp>
namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARCornerRefinementOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARCornerRefinementOpencv::SolARCornerRefinementOpencv() :ConfigurableBase(xpcf::toUUID<SolARCornerRefinementOpencv>())
{
    declareInterface<api::features::ICornerRefinement>(this);
	declareProperty("nbMaxIters", m_nbMaxIters);
	declareProperty("minAccuracy", m_minAccuracy);
	declareProperty("winSize", m_winSize);
	LOG_DEBUG(" SolARFiducialMarkerPoseEstimatorOpencv constructor")
}

xpcf::XPCFErrorCode SolARCornerRefinementOpencv::onConfigured()
{
	m_termcrit = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, m_nbMaxIters, m_minAccuracy);
	m_subPixWinSize = cv::Size(m_winSize, m_winSize);
	LOG_DEBUG(" SolARCornerRefinementOpencv configured");
	return xpcf::XPCFErrorCode::_SUCCESS;
}

void SolARCornerRefinementOpencv::refine(const SRef<datastructure::Image> image, std::vector<datastructure::Point2Df>& corners)
{
	// transform all SolAR data to openCv data
	SRef<Image> convertedImage = image;
	if (image->getImageLayout() != Image::ImageLayout::LAYOUT_GREY) {
		// input Image not in grey levels : convert it !
		SolARImageConvertorOpencv convertor;
		convertedImage = xpcf::utils::make_shared<Image>(Image::ImageLayout::LAYOUT_GREY, Image::PixelOrder::INTERLEAVED, image->getDataType());
		convertor.convert(image, convertedImage);
	}
	cv::Mat imageOpencv;
	SolAROpenCVHelper::mapToOpenCV(convertedImage, imageOpencv);

	std::vector<cv::Point2f> cornersOpencv;
	for (const auto &cor : corners)
		cornersOpencv.push_back(cv::Point2f(cor.getX(), cor.getY()));

	// refine
	cv::cornerSubPix(imageOpencv, cornersOpencv, m_subPixWinSize, cv::Size(-1, -1), m_termcrit);

	// return output
	corners.clear();
	for (const auto &cor : cornersOpencv)
		corners.push_back(datastructure::Point2Df(cor.x, cor.y));
}

}
}
}

