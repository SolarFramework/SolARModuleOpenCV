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

#include "SolARStereoRectificationOpencv.h"
#include "core/Log.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/core/eigen.hpp"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARStereoRectificationOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARStereoRectificationOpencv::SolARStereoRectificationOpencv() :ConfigurableBase(xpcf::toUUID<SolARStereoRectificationOpencv>())
{
	declareInterface<api::image::IRectification>(this);
	LOG_DEBUG("SolARStereoRectificationOpencv constructor");
}

SolARStereoRectificationOpencv::~SolARStereoRectificationOpencv()
{
	LOG_DEBUG("SolARStereoRectificationOpencv destructor");
}

datastructure::Point2Df SolARStereoRectificationOpencv::rectifyPoint(const SolAR::datastructure::Point2Df & pt2D, const SolAR::datastructure::RectificationParameters& rectParams)
{
	float u = pt2D.getX();
	float v = pt2D.getY();
	float x = (u - rectParams.camParams.intrinsic(0, 2)) / rectParams.camParams.intrinsic(0, 0);
	float y = (v - rectParams.camParams.intrinsic(1, 2)) / rectParams.camParams.intrinsic(1, 1);
	Maths::Vector3f pt3D(x, y, 1);
	Maths::Vector3f pt3DRect = rectParams.rotation * pt3D;
	float xRect = pt3DRect(0) / pt3DRect(2);
	float yRect = pt3DRect(1) / pt3DRect(2);
	float uRect = xRect * rectParams.projection(0, 0) + rectParams.projection(0, 2);
	float vRect = yRect * rectParams.projection(1, 1) + rectParams.projection(1, 2);
	return Point2Df(uRect, vRect);
}

FrameworkReturnCode SolARStereoRectificationOpencv::rectify(SRef<SolAR::datastructure::Image> image, const SolAR::datastructure::RectificationParameters& rectParams, SRef<SolAR::datastructure::Image>& rectifiedImage)
{
	cv::Mat cvImage = SolAROpenCVHelper::mapToOpenCV(image);
	cv::Mat intrinsic, distortion, R, P;
	cv::eigen2cv(rectParams.camParams.intrinsic, intrinsic);
	cv::eigen2cv(rectParams.camParams.distortion, distortion);
	cv::eigen2cv(rectParams.rotation, R);
	cv::eigen2cv(rectParams.projection, P);
	cv::Mat map1, map2;
	cv::initUndistortRectifyMap(intrinsic, distortion, R, P, cvImage.size(), CV_16SC2, map1, map2);
	cv::Mat cvImageRectified;
	cv::remap(cvImage, cvImageRectified, map1, map2, cv::INTER_LINEAR);
	SolAROpenCVHelper::convertToSolar(cvImageRectified, rectifiedImage);
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARStereoRectificationOpencv::rectify(const std::vector<SolAR::datastructure::Point2Df>& points2D, const SolAR::datastructure::RectificationParameters& rectParams, std::vector<SolAR::datastructure::Point2Df>& rectifiedPoints2D)
{
	for (int i = 0; i < points2D.size(); ++i) {
		Point2Df rectPoint2D = rectifyPoint(points2D[i], rectParams);
		rectifiedPoints2D.push_back(rectPoint2D);
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARStereoRectificationOpencv::rectify(const std::vector<SolAR::datastructure::Keypoint>& keypoints, const SolAR::datastructure::RectificationParameters& rectParams, std::vector<SolAR::datastructure::Keypoint>& rectifiedKeypoints)
{
	for (int i = 0; i < keypoints.size(); ++i) {
		Point2Df rectPoint2D = rectifyPoint(keypoints[i], rectParams);
		Keypoint rectKeypoint = keypoints[i];
		rectKeypoint.setX(rectPoint2D.getX());
		rectKeypoint.setY(rectPoint2D.getY());
		rectifiedKeypoints.push_back(rectKeypoint);
	}
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}