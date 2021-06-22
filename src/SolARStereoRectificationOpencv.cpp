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
	declareInterface<api::stereo::IStereoRectification>(this);
	declareProperty("rectificationFile", m_rectificationFile);
	LOG_DEBUG("SolARStereoRectificationOpencv constructor");
}

SolARStereoRectificationOpencv::~SolARStereoRectificationOpencv()
{
	LOG_DEBUG("SolARStereoRectificationOpencv destructor");
}

xpcf::XPCFErrorCode SolARStereoRectificationOpencv::onConfigured()
{
	LOG_DEBUG("SolARStereoRectificationOpencv onConfigured");
	if (m_rectificationFile.empty())
	{
		LOG_ERROR("Stereo rectification file path is empty");
		return xpcf::XPCFErrorCode::_FAIL;
	}
	cv::FileStorage fs(m_rectificationFile, cv::FileStorage::READ);	
	if (!fs.isOpened()){
		LOG_ERROR("Cannot open stereo rectification file ");
		return xpcf::XPCFErrorCode::_FAIL;
	}
	m_R.resize(2);
	m_P.resize(2);
	m_intrinsic.resize(2);
	m_distortion.resize(2);
	m_rectificationParams.resize(2);
	fs["image_width"] >> m_imageSize.width;
	fs["image_height"] >> m_imageSize.height;
	fs["camera_matrix1"] >> m_intrinsic[0];
	fs["distortion_coefficients1"] >> m_distortion[0];
	fs["rotation_matrix1"] >> m_R[0];
	fs["projection_matrix1"] >> m_P[0];
	fs["camera_matrix2"] >> m_intrinsic[1];
	fs["distortion_coefficients2"] >> m_distortion[1];	
	fs["rotation_matrix2"] >> m_R[1];	
	fs["projection_matrix2"] >> m_P[1];	
	for (int i = 0; i < 2; ++i) {
		cv::cv2eigen(m_R[i], m_rectificationParams[i].rotation);
		cv::cv2eigen(m_P[i], m_rectificationParams[i].projection);
	}
	if (m_rectificationParams[1].projection(0, 3) != 0) {
		m_baseline = std::abs(m_rectificationParams[1].projection(0, 3) / m_rectificationParams[1].projection(0, 0));
		m_stereoType = StereoType::Horizontal;
	}
	else {
		m_baseline = std::abs(m_rectificationParams[1].projection(1, 3) / m_rectificationParams[1].projection(0, 0));
		m_stereoType = StereoType::Vertical;
	}
	return xpcf::XPCFErrorCode::_SUCCESS;
}

datastructure::Point2Df SolARStereoRectificationOpencv::rectifyPoint(const SolAR::datastructure::Point2Df & pt2D, int indexCamera)
{
	float u = pt2D.getX();
	float v = pt2D.getY();
	float x = (u - m_intrinsic[indexCamera].at<double>(0, 2)) / m_intrinsic[indexCamera].at<double>(0, 0);
	float y = (v - m_intrinsic[indexCamera].at<double>(1, 2)) / m_intrinsic[indexCamera].at<double>(1, 1);
	cv::Point3d pt3D(x, y, 1);
	cv::Mat pt3DRect = m_R[indexCamera] * cv::Mat(pt3D);
	float xRect = pt3DRect.at<double>(0) / pt3DRect.at<double>(2);
	float yRect = pt3DRect.at<double>(1) / pt3DRect.at<double>(2);
	float uRect = xRect * m_P[indexCamera].at<double>(0, 0) + m_P[indexCamera].at<double>(0, 2);
	float vRect = yRect * m_P[indexCamera].at<double>(1, 1) + m_P[indexCamera].at<double>(1, 2);
	return Point2Df(uRect, vRect);
}

void SolARStereoRectificationOpencv::rectify(SRef<SolAR::datastructure::Image> image, SRef<SolAR::datastructure::Image>& rectifiedImage, int indexCamera)
{
	assert((indexCamera == 0 || indexCamera == 1) && "Camera index must be 0 or 1");
	cv::Mat cvImage = SolAROpenCVHelper::mapToOpenCV(image);
	cv::Mat map1, map2;
	cv::initUndistortRectifyMap(m_intrinsic[indexCamera], m_distortion[indexCamera], m_R[indexCamera], m_P[indexCamera], cvImage.size(), CV_16SC2, map1, map2);
	cv::Mat cvImageRectified;
	cv::remap(cvImage, cvImageRectified, map1, map2, cv::INTER_LINEAR);
	SolAROpenCVHelper::convertToSolar(cvImageRectified, rectifiedImage);
}

void SolARStereoRectificationOpencv::rectify(const std::vector<SolAR::datastructure::Point2Df>& points2D, std::vector<SolAR::datastructure::Point2Df>& rectifiedPoints2D, int indexCamera)
{
	assert((indexCamera == 0 || indexCamera == 1) && "Camera index must be 0 or 1");
	for (int i = 0; i < points2D.size(); ++i) {
		Point2Df rectPoint2D = rectifyPoint(points2D[i], indexCamera);
		rectifiedPoints2D.push_back(rectPoint2D);
	}
}

void SolARStereoRectificationOpencv::rectify(const std::vector<SolAR::datastructure::Keypoint>& keypoints, std::vector<SolAR::datastructure::Keypoint>& rectifiedKeypoints, int indexCamera)
{
	assert((indexCamera == 0 || indexCamera == 1) && "Camera index must be 0 or 1");
	for (int i = 0; i < keypoints.size(); ++i) {
		Point2Df rectPoint2D = rectifyPoint(keypoints[i], indexCamera);
		Keypoint rectKeypoint = keypoints[i];
		rectKeypoint.setX(rectPoint2D.getX());
		rectKeypoint.setY(rectPoint2D.getY());
		rectifiedKeypoints.push_back(rectKeypoint);
	}
}

SolAR::datastructure::StereoType SolARStereoRectificationOpencv::getType()
{
	return m_stereoType;
}

float SolARStereoRectificationOpencv::getBaseline()
{
	return m_baseline;
}

SolAR::datastructure::RectificationParameters SolARStereoRectificationOpencv::getRectificationParamters(int indexCamera)
{
	assert((indexCamera == 0 || indexCamera == 1) && "Camera index must be 0 or 1");
	return m_rectificationParams[indexCamera];
}

}
}
}