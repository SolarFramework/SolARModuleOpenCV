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

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARStereoCalibrationOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARStereoCalibrationOpencv::SolARStereoCalibrationOpencv() :ComponentBase(xpcf::toUUID<SolARStereoCalibrationOpencv>())
{
	declareInterface<api::stereo::IStereoCalibration>(this);
	LOG_DEBUG("SolARStereoCalibrationOpencv constructor");
}


SolARStereoCalibrationOpencv::~SolARStereoCalibrationOpencv()
{
	LOG_DEBUG("SolARStereoCalibrationOpencv destructor");
}

FrameworkReturnCode SolARStereoCalibrationOpencv::calibrate(const std::vector<SRef<SolAR::datastructure::Image>>& images1, const std::vector<SRef<SolAR::datastructure::Image>>& images2, const std::string & calibrationFilePath)
{
	if (images1.size() != images2.size()) {
		LOG_ERROR("The number of images from the two cameras must be the same");
		return FrameworkReturnCode::_ERROR_;
	}
	if (!m_isSetConfig || !m_isSetParams1 || !m_isSetParams2) {
		LOG_ERROR("Must set all configurations and camera parameters before calibrating stereo camera");
		return FrameworkReturnCode::_ERROR_;
	}
	std::vector<std::vector<cv::Point2f>> imagePoints1, imagePoints2;
	SRef<Image> image1, image2;
	for (int i = 0; i < images1.size(); ++i){
		// get image of two cameras
		image1 = images1[i];
		image2 = images2[i];
		cv::Mat displayImage1, displayImage2;
		std::vector<cv::Point2f> pts1, pts2;
		bool isFoundCorners = findChessboardCornersImage(image1, image2, displayImage1, displayImage2, pts1, pts2);
		if ((i % m_nbDropFrames == 0) && isFoundCorners) {
			imagePoints1.push_back(pts1);
			imagePoints2.push_back(pts2);
		}
		// display
		cv::imshow("Image 1", displayImage1);
		cv::imshow("Image 2", displayImage2);
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

	// save to file
	cv::FileStorage fs(calibrationFilePath, cv::FileStorage::WRITE);
	fs << "image_width" << m_imageSize.width;
	fs << "image_height" << m_imageSize.height;
	fs << "board_width" << m_boardSize.width;
	fs << "board_height" << m_boardSize.height;
	fs << "square_size" << m_squareSize;
	fs << "camera_matrix1" << m_intrinsic1;	
	fs << "distortion_coefficients1" << m_distortion1;
	fs << "camera_matrix2" << m_intrinsic2;
	fs << "distortion_coefficients2" << m_distortion2;
	fs << "rotation_matrix" << R;
	fs << "translation_vector" << T;
	fs << "essential_matrix" << E;
	fs << "fundamental_matrix" << F;
	fs.release();
	LOG_INFO("Stereo calibration done!");	
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARStereoCalibrationOpencv::rectify(const std::string & calibrationFilePath, const std::string & rectificationFilePath)
{
	// load calibration parameters
	cv::FileStorage fs(calibrationFilePath, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		LOG_ERROR("Cannot open calibration file");
		return FrameworkReturnCode::_ERROR_;
	}
	cv::Size imageSize;
	cv::Mat intrinsic1, intrinsic2, distortion1, distortion2, R;
	cv::Vec3d T;
	fs["image_width"] >> imageSize.width;
	fs["image_height"] >> imageSize.height;
	fs["camera_matrix1"] >> intrinsic1;
	fs["distortion_coefficients1"] >> distortion1;
	fs["camera_matrix2"] >> intrinsic2;
	fs["distortion_coefficients2"] >> distortion2;
	fs["rotation_matrix"] >> R;
	fs["translation_vector"] >> T;
	fs.release();

	// rectification
	LOG_INFO("Stereo rectifying...");
	cv::Mat R1, R2, P1, P2, Q;
	cv::stereoRectify(intrinsic1, distortion1, intrinsic2, distortion2, imageSize, R, T, R1, R2, P1, P2, Q);
	cv::FileStorage fsRect(rectificationFilePath, cv::FileStorage::WRITE);
	fsRect << "image_width" << imageSize.width;
	fsRect << "image_height" << imageSize.height;
	fsRect << "camera_matrix1" << intrinsic1;
	fsRect << "distortion_coefficients1" << distortion1;
	fsRect << "camera_matrix2" << intrinsic2;
	fsRect << "distortion_coefficients2" << distortion2;
	fsRect << "rotation_matrix1" << R1;
	fsRect << "projection_matrix1" << P1;
	fsRect << "rotation_matrix2" << R2;	
	fsRect << "projection_matrix2" << P2;
	fsRect.release();
	LOG_INFO("Rectification done!");
	return FrameworkReturnCode::_SUCCESS;
}

void SolARStereoCalibrationOpencv::setCameraParameters1(const SolAR::datastructure::CameraParameters & camParams)
{	
	m_intrinsic1 = cv::Mat(3, 3, CV_32FC1, (void *)camParams.intrinsic.data());
	m_distortion1 = cv::Mat(5, 1, CV_32FC1, (void *)camParams.distortion.data());
	m_isSetParams1 = true;
}

void SolARStereoCalibrationOpencv::setCameraParameters2(const SolAR::datastructure::CameraParameters & camParams)
{
	m_intrinsic2 = cv::Mat(3, 3, CV_32FC1, (void *)camParams.intrinsic.data());
	m_distortion2 = cv::Mat(5, 1, CV_32FC1, (void *)camParams.distortion.data());
	m_isSetParams2 = true;
}

void SolARStereoCalibrationOpencv::setConfiguration(const std::string & configFile)
{
	cv::FileStorage fs(configFile, cv::FileStorage::READ);	
	if (fs.isOpened())
	{
		m_boardSize.width = (int)fs["chessboard_width"];
		m_boardSize.height = (int)fs["chessboard_height"];
		m_squareSize = (float)fs["square_size"];
		m_nbFrames = (int)fs["nb_frames"];
		m_nbDropFrames = (int)fs["nb_drop_frames"];
		m_waitTime = (int)fs["nb_wait_time"];
		fs.release();
		LOG_DEBUG("Configuration parameters for stereo calibration:");
		LOG_DEBUG("    ->board size: {}", m_boardSize);
		LOG_DEBUG("    ->square size: {}", m_squareSize);
		LOG_DEBUG("    ->nb frames: {}", m_nbFrames);
		LOG_DEBUG("    ->nb drop frames: {}", m_nbDropFrames);
		m_isSetConfig = true;
	}
	else
	{
		LOG_ERROR("Cannot open configuration file: {}", configFile);
	}
}

bool SolARStereoCalibrationOpencv::findChessboardCornersImage(SRef<Image>& image1, SRef<Image> image2, cv::Mat & displayImage1, cv::Mat & displayImage2, std::vector<cv::Point2f>& corners1, std::vector<cv::Point2f>& corners2)
{
	// convert to opencv image
	cv::Mat cvImage1 = SolAROpenCVHelper::mapToOpenCV(image1);
	cv::Mat cvImage2 = SolAROpenCVHelper::mapToOpenCV(image2);
	cvImage1.copyTo(displayImage1);
	cvImage2.copyTo(displayImage2);
	cv::Mat cvImage1Gray, cvImage2Gray;
	// convert to gray image
	if (cvImage1.channels() == 3)
		cv::cvtColor(cvImage1, cvImage1Gray, cv::COLOR_BGR2GRAY);
	else
		cvImage1Gray = cvImage1;
	if (cvImage2.channels() == 3)
		cv::cvtColor(cvImage2, cvImage2Gray, cv::COLOR_BGR2GRAY);
	else
		cvImage2Gray = cvImage2;

	// detect corners
	bool found1 = cv::findChessboardCorners(cvImage1Gray, m_boardSize, corners1, cv::CALIB_CB_ADAPTIVE_THRESH);
	bool found2 = cv::findChessboardCorners(cvImage2Gray, m_boardSize, corners2, cv::CALIB_CB_ADAPTIVE_THRESH);

	// improve the found corners coordinate accuracy
	if (found1 && found2) {
		cornerSubPix(cvImage1Gray, corners1, cv::Size(11, 11),
			cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
		cornerSubPix(cvImage2Gray, corners2, cv::Size(11, 11),
			cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
		cv::drawChessboardCorners(displayImage1, m_boardSize, cv::Mat(corners1), found1);
		cv::drawChessboardCorners(displayImage2, m_boardSize, cv::Mat(corners2), found2);
		return true;
	}
	return false;
}

}
}
}