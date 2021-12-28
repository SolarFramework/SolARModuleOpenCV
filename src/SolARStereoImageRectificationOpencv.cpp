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

#include "SolARStereoImageRectificationOpencv.h"
#include "core/Log.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/core/eigen.hpp"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARStereoImageRectificationOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARStereoImageRectificationOpencv::SolARStereoImageRectificationOpencv() :ConfigurableBase(xpcf::toUUID<SolARStereoImageRectificationOpencv>())
{
    declareInterface<api::image::IImageRectification>(this);
    LOG_DEBUG("SolARStereoImageRectificationOpencv constructor");
}

SolARStereoImageRectificationOpencv::~SolARStereoImageRectificationOpencv()
{
    LOG_DEBUG("SolARStereoImageRectificationOpencv destructor");
}

FrameworkReturnCode SolARStereoImageRectificationOpencv::rectify(SRef<SolAR::datastructure::Image> image,
															const SolAR::datastructure::CameraParameters& camParams,
															const SolAR::datastructure::RectificationParameters& rectParams, 
															SRef<SolAR::datastructure::Image>& rectifiedImage)
{
	cv::Mat cvImage = SolAROpenCVHelper::mapToOpenCV(image);
	cv::Mat intrinsic, distortion, R, P;
	cv::eigen2cv(camParams.intrinsic, intrinsic);
	cv::eigen2cv(camParams.distortion, distortion);
	cv::eigen2cv(rectParams.rotation, R);
	cv::eigen2cv(rectParams.projection, P);
	cv::Mat map1, map2;
	cv::initUndistortRectifyMap(intrinsic, distortion, R, P, cvImage.size(), CV_16SC2, map1, map2);
	cv::Mat cvImageRectified;
	cv::remap(cvImage, cvImageRectified, map1, map2, cv::INTER_LINEAR);
	SolAROpenCVHelper::convertToSolar(cvImageRectified, rectifiedImage);
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}
