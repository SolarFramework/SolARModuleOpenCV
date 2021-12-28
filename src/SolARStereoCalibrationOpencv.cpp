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

#include "SolARStereoCalibrationOpencv.h"
#include "core/Log.h"
#include "SolAROpenCVHelper.h"
#include <cassert>
#include "opencv2/core/eigen.hpp"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARStereoCalibrationOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARStereoCalibrationOpencv::SolARStereoCalibrationOpencv() :ConfigurableBase(xpcf::toUUID<SolARStereoCalibrationOpencv>())
{
	declareInterface<api::input::devices::IStereoCameraCalibration>(this);
	declareProperty("chessboard_width", m_boardSize.width);
	declareProperty("chessboard_height", m_boardSize.height);
	declareProperty("square_size", m_squareSize);
	declareProperty("nb_frames", m_nbFrames);
	declareProperty("nb_drop_frames", m_nbDropFrames);
	declareProperty("nb_wait_time", m_waitTime);
	LOG_DEBUG("SolARStereoCalibrationOpencv constructor");
}


SolARStereoCalibrationOpencv::~SolARStereoCalibrationOpencv()
{
	LOG_DEBUG("SolARStereoCalibrationOpencv destructor");
}

FrameworkReturnCode SolARStereoCalibrationOpencv::calibrate(const std::vector<SRef<SolAR::datastructure::Image>>& images1, 
															const std::vector<SRef<SolAR::datastructure::Image>>& images2, 
															const SolAR::datastructure::CameraParameters & camParams1, 
															const SolAR::datastructure::CameraParameters & camParams2, 
															SolAR::datastructure::Transform3Df & transformation, 
															SolAR::datastructure::RectificationParameters & rectParams1, 
															SolAR::datastructure::RectificationParameters & rectParams2)
{
	m_intrinsic1 = cv::Mat(3, 3, CV_32FC1, (void *)camParams1.intrinsic.data());
	m_distortion1 = cv::Mat(5, 1, CV_32FC1, (void *)camParams1.distortion.data());
	m_intrinsic2 = cv::Mat(3, 3, CV_32FC1, (void *)camParams2.intrinsic.data());
	m_distortion2 = cv::Mat(5, 1, CV_32FC1, (void *)camParams2.distortion.data());

	if ((images1.size() != images2.size()) || images1.size() == 0) {
		LOG_ERROR("The number of images from the two cameras must be the same");
		return FrameworkReturnCode::_ERROR_;
	}
	if (m_nbDropFrames < 0) {
		LOG_ERROR("Number of drop images must be greater than or equal to zero");
		return FrameworkReturnCode::_ERROR_;
	}

	std::vector<std::vector<cv::Point2f>> imagePoints1, imagePoints2;
	SRef<Image> image1, image2;
	for (int i = 0; i < images1.size(); ++i) {
		// get image of two cameras
		image1 = images1[i];
		image2 = images2[i];
		cv::Mat displayImage1, displayImage2;
		std::vector<cv::Point2f> pts1, pts2;
		bool isFoundCorners1 = findChessboardCornersImage(image1, displayImage1, pts1);
		bool isFoundCorners2 = findChessboardCornersImage(image2, displayImage2, pts2);
		if ((i % (m_nbDropFrames + 1) == 0) && isFoundCorners1 && isFoundCorners2) {
			imagePoints1.push_back(pts1);
			imagePoints2.push_back(pts2);
		}
		// display
		cv::imshow("Image view 1", displayImage1);
		cv::imshow("Image view 2", displayImage2);
		cv::waitKey(m_waitTime);
		// exit for if number of frames if enough
		if (imagePoints1.size() >= m_nbFrames)
			break;
	}
	if (imagePoints1.size() < m_nbFrames) {
		LOG_ERROR("Not enough data before calibrating stereo camera");
		return FrameworkReturnCode::_ERROR_;
	}
	// get object points (points 3D of chessboard)
	std::vector<std::vector<cv::Point3f>> objectPoints;
	std::vector<cv::Point3f> corners;
	for (int i = 0; i < m_boardSize.height; i++)
		for (int j = 0; j < m_boardSize.width; j++)
			corners.push_back(cv::Point3f(float(j * m_squareSize), float(i * m_squareSize), 0));
	objectPoints.resize(imagePoints1.size(), corners);

	// stereo calibrate
	cv::Mat R, F, E;
	cv::Vec3d T;
	LOG_INFO("Stereo calibrating...");
	cv::stereoCalibrate(objectPoints, imagePoints1, imagePoints2, m_intrinsic1, m_distortion1, m_intrinsic2, m_distortion2,
		cv::Size(image1->getWidth(), image1->getHeight()), R, T, E, F, cv::CALIB_FIX_INTRINSIC);
	m_imageSize.width = image1->getWidth();
	m_imageSize.height = image1->getHeight();
	LOG_INFO("Calibration done!");
	// create transformation
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j)
			transformation(i, j) = R.at<double>(i, j);
		transformation(i, 3) = T[i];
	}
	// rectification
	LOG_INFO("Stereo rectifying...");
	cv::Mat R1, R2, P1, P2, Q;
	cv::stereoRectify(m_intrinsic1, m_distortion1, m_intrinsic2, m_distortion2, m_imageSize, R, T, R1, R2, P1, P2, Q);
	LOG_INFO("Rectification done!");
	// create rectifications
	cv::cv2eigen(R1, rectParams1.rotation);
	cv::cv2eigen(P1, rectParams1.projection);
	cv::cv2eigen(R2, rectParams2.rotation);
	cv::cv2eigen(P2, rectParams2.projection);
	float fb, baseline;
	StereoType type;
	if (rectParams2.projection(0, 3) != 0) {
		fb = std::abs(rectParams2.projection(0, 3));
		type = StereoType::Horizontal;
	}
	else {
		fb = std::abs(rectParams2.projection(1, 3));
		type = StereoType::Vertical;
	}
	baseline = fb / rectParams2.projection(0, 0);
	rectParams1.baseline = baseline;
	rectParams2.baseline = baseline;
	rectParams1.type = type;
	rectParams2.type = type;
	return FrameworkReturnCode::_SUCCESS;
}

bool SolARStereoCalibrationOpencv::findChessboardCornersImage(SRef<Image>& image, cv::Mat & displayImage, std::vector<cv::Point2f>& corners)
{
	// convert to opencv image
	cv::Mat cvImage = SolAROpenCVHelper::mapToOpenCV(image);
	cvImage.copyTo(displayImage);
	cv::Mat cvImageGray;
	// convert to gray image
	if (cvImage.channels() == 3)
		cv::cvtColor(cvImage, cvImageGray, cv::COLOR_BGR2GRAY);
	else
		cvImageGray = cvImage;

	// detect corners
	bool found = cv::findChessboardCorners(cvImageGray, m_boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH);

	// improve the found corners coordinate accuracy
	if (found) {
		cornerSubPix(cvImageGray, corners, cv::Size(11, 11),
			cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
		cv::drawChessboardCorners(displayImage, m_boardSize, cv::Mat(corners), found);
		return true;
	}
	return false;
}

}
}
}