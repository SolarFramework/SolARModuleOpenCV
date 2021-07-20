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

#include "SolARCameraCalibrationOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/eigen.hpp"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARCameraCalibrationOpencv)

namespace SolAR {
namespace MODULES {
namespace OPENCV {

SolARCameraCalibrationOpencv::SolARCameraCalibrationOpencv():ConfigurableBase(xpcf::toUUID<SolARCameraCalibrationOpencv>())
{
    declareInterface<api::input::devices::ICameraCalibration>(this);
	declareProperty("image_width", m_imageSize.width);
	declareProperty("image_height", m_imageSize.height);
	declareProperty("chessboard_width", m_boardSize.width);
	declareProperty("chessboard_height", m_boardSize.height);
	declareProperty("square_size", m_squareSize);
	declareProperty("nb_frames", m_nbFrames);
	declareProperty("nb_drop_frames", m_nbDropFrames);
	declareProperty("nb_wait_time", m_waitTime);
	declareProperty("flags", m_flags);	
	m_camMatrix.create(3, 3, CV_32FC1);
	m_camDistortion.create(5, 1, CV_32FC1);
	LOG_DEBUG("SolARCameraCalibrationOpencv constructor")
}


SolARCameraCalibrationOpencv::~SolARCameraCalibrationOpencv()
{
    LOG_DEBUG("SolARCameraCalibrationOpencv destructor")
}

FrameworkReturnCode SolARCameraCalibrationOpencv::calibrate(const std::vector<SRef<SolAR::datastructure::Image>>& images, SolAR::datastructure::CameraParameters & camParams)
{
	if (images.size() == 0)
		return FrameworkReturnCode::_ERROR_;

	SRef<datastructure::Image> image;
	std::vector<std::vector<cv::Point2f> > imagePoints;
	for (int i = 0; i < images.size(); ++i) {
		image = images[i];
		cv::Mat displayImage;
		std::vector<cv::Point2f> imagePts;
		// detect chessboard corners
		bool isFoundCorners = findChessboardCornersImage(image, displayImage, imagePts);
		if ((i % (m_nbDropFrames + 1) == 0) && isFoundCorners) {
			imagePoints.push_back(imagePts);
			cv::bitwise_not(displayImage, displayImage);
		}
		// display
		cv::imshow("Image View", displayImage);
		cv::waitKey(m_waitTime);
		// exit for if number of frames if enough
		if (imagePoints.size() >= m_nbFrames)
			break;
	}
	if (imagePoints.size() < m_nbFrames) {
		LOG_ERROR("Not enough data before calibrating camera");
		return FrameworkReturnCode::_ERROR_;
	}
	// get object points (points 3D of chessboard)
	std::vector<std::vector<cv::Point3f>> objectPoints;
	std::vector<cv::Point3f> corners;
	for (int i = 0; i < m_boardSize.height; i++)
		for (int j = 0; j < m_boardSize.width; j++)
			corners.push_back(cv::Point3f(float(j * m_squareSize), float(i * m_squareSize), 0));
	objectPoints.resize(imagePoints.size(), corners);
	// calibrate
	LOG_INFO("Calibrating...");
	std::vector<cv::Mat> rvecs, tvecs;
	std::vector<float> reprojErrs;
	double rms = cv::calibrateCamera(objectPoints, imagePoints, cv::Size(image->getWidth(), image->getHeight()), 
		m_camMatrix, m_camDistortion, rvecs, tvecs, m_flags | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
	bool ok = cv::checkRange(m_camMatrix) && cv::checkRange(m_camDistortion);
	LOG_INFO("{} avg reprojection error = {:.2f}", ok ? "Calibration succeeded" : "Calibration failed", rms);

	if (!ok)
		return FrameworkReturnCode::_ERROR_;
	// create camera parameters datastructure
	camParams.resolution.width = m_imageSize.width;
	camParams.resolution.height = m_imageSize.height;
	std::cout << "intrinsic: " << std::endl << m_camMatrix << std::endl;
	std::cout << "distortion: " << std::endl << m_camDistortion << std::endl;
	cv::cv2eigen(m_camMatrix, camParams.intrinsic);
	cv::cv2eigen(m_camDistortion, camParams.distortion);

	return FrameworkReturnCode::_SUCCESS;
}

bool SolARCameraCalibrationOpencv::findChessboardCornersImage(SRef<SolAR::datastructure::Image>& image, cv::Mat & displayImage, std::vector<cv::Point2f>& corners)
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
}  // end of namespace Solar


