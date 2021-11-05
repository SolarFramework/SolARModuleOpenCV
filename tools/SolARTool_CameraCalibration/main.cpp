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
#include <iostream>
#include <boost/log/core.hpp>
#include "core/Log.h"
#include "datastructure/CameraDefinitions.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"

using namespace SolAR;
using namespace SolAR::datastructure;

const cv::String keys =
"{help h usage ?||}"
"{config c|calibration_config.yml| calibration configuration file}"
"{input i|0| index of camera or a video file}"
"{output o|camera_calibration.json| camera calibration output file in json format}"
;

enum ProcessMode
{
	SOLAR_DETECT = 0,
	SOLAR_CAPTURE = 1,
	SOLAR_CALIBRATED = 2
};

// compute error function
double computeReprojectionErrors(
	const std::vector<std::vector<cv::Point3f> >& objectPoints,
	const std::vector<std::vector<cv::Point2f> >& imagePoints,
	const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
	const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
	std::vector<float>& perViewErrors);

// save camera parameters
void saveCameraParams(
	const cv::Size& imageSize,
	const cv::Mat& cameraMatrix,
	const cv::Mat& distCoeffs,
	const std::string& outFile);

int main(int argc, char *argv[])
{
#if NDEBUG
	boost::log::core::get()->set_logging_enabled(false);
#endif
	LOG_ADD_LOG_TO_CONSOLE();
	cv::CommandLineParser parser(argc, argv, keys);
	if (parser.has("help"))
	{
		parser.printMessage();
		return 0;
	}

	// get parameters
	if (!parser.has("config")) 
	{
		LOG_ERROR("Must give a configuration file");
		return -1;
	}
	std::string configFile = parser.get<std::string>("config");
	if (!parser.has("input")) 
	{
		LOG_ERROR("Must give an index of camera or a video file");
		return -1;
	}
	std::string input = parser.get<std::string>("input");
	if (!parser.has("output")) 
	{
		LOG_ERROR("Must give a camera calibration output file");
		return -1;
	}
	std::string outputFile = parser.get<std::string>("output");

	// Load configuration file
	cv::FileStorage fs(configFile, cv::FileStorage::READ);	
	cv::Size boardSize;
	float squareSize, aspectRatio;
	int nbFrames, flags, delay;
	if (fs.isOpened())
	{
		boardSize.width = (int)fs["chessboard_width"];
		boardSize.height = (int)fs["chessboard_height"];
		squareSize = (float)fs["square_size"];
		aspectRatio = (float)fs["aspect_ratio"];
		nbFrames = (int)fs["nb_frames"];
		flags = (int)fs["flags"];
		delay = (int)fs["delay"];
		fs.release();
		LOG_DEBUG("Camera calibration parameters:");
		LOG_DEBUG("    ->board size: {}", boardSize);
		LOG_DEBUG("    ->square size: {}", squareSize);
		LOG_DEBUG("    ->aspect ratio: {}", aspectRatio);
		LOG_DEBUG("    ->nb frames: {}", nbFrames);
		LOG_DEBUG("    ->flags: {}", flags);
		LOG_DEBUG("    ->delay: {}", delay);

	}
	else
	{
		LOG_ERROR("Calibration config file {} does not exist", configFile)
		return -1;
	}

	// create 3D corner points
	std::vector<cv::Point3f> corners;
	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
	
	// Load input
	cv::VideoCapture capture;
	bool isOpen;
	if (input.length() > 1)
		isOpen = capture.open(input);
	else
		isOpen = capture.open(std::stoi(input));
	
	if (!isOpen) 
	{
		LOG_ERROR("Cannot open input {}", input);
		return -1;
	}

	// Calibrate process
	cv::Size imageSize;
	clock_t prevTimestamp = 0;
	std::vector<std::vector<cv::Point2f> > imagePoints;
	ProcessMode mode = SOLAR_DETECT;

	while (true) 
	{
		cv::Mat image, imageGray;
		bool blink = false;
		// get image
		if (!capture.read(image))
			break;
		imageSize = image.size();
		// convert to gray image
		cv::cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);
		std::vector<cv::Point2f> pointbuf;
		bool found = cv::findChessboardCorners(image, boardSize, pointbuf, cv::CALIB_CB_ADAPTIVE_THRESH);

		// improve the found corners' coordinate accuracy
		if (found) cornerSubPix(imageGray, pointbuf, cv::Size(11, 11),
			cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

		if (mode == SOLAR_CAPTURE && found &&
			(!capture.isOpened() || clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC)) {
			imagePoints.push_back(pointbuf);
			prevTimestamp = clock();
			blink = capture.isOpened();
		}

		if (found)
			cv::drawChessboardCorners(image, boardSize, cv::Mat(pointbuf), found);

		std::string msg = mode == SOLAR_CAPTURE ? "100/100" :
			mode == SOLAR_CALIBRATED ? "Calibrated" : "Press 'g' to start";
		int baseLine = 0;
		cv::Size textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
		cv::Point textOrigin(image.cols - 2 * textSize.width - 10, image.rows - 2 * baseLine - 10);

		if (mode == SOLAR_CAPTURE)
			msg = cv::format("%d/%d", (int)imagePoints.size(), nbFrames);

		cv::putText(image, msg, textOrigin, 1, 1,
			mode != SOLAR_CALIBRATED ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));

		if (blink)
			cv::bitwise_not(image, image);

		cv::imshow("Image View", image);
		char key = cv::waitKey(30);
		if (key == 27)
			break;
		else if (key == 'g') {
			mode = SOLAR_CAPTURE;
			imagePoints.clear();
		}

		if (mode == SOLAR_CAPTURE && imagePoints.size() >= (unsigned)nbFrames) {
			std::vector<cv::Mat> rvecs, tvecs;
			std::vector<float> reprojErrs;
			cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
			if (flags & cv::CALIB_FIX_ASPECT_RATIO)
				cameraMatrix.at<double>(0, 0) = aspectRatio;
			cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

			std::vector<std::vector<cv::Point3f> > objectPoints;
			objectPoints.resize(imagePoints.size(), corners);

			double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
				distCoeffs, rvecs, tvecs, flags | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);

			bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);
			double totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
				rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

			if (ok) {
				saveCameraParams(imageSize, cameraMatrix, distCoeffs, outputFile);
				LOG_INFO("Calibration succeeded with avg reprojection error = {:.2f}", totalAvgErr);
				LOG_INFO("Please find the camera parameters in {} file", outputFile);
				mode = SOLAR_CALIBRATED;
			}				
			else
				mode = SOLAR_DETECT;
		}
	}

    return 0;
}

double computeReprojectionErrors(
	const std::vector<std::vector<cv::Point3f> >& objectPoints,
	const std::vector<std::vector<cv::Point2f> >& imagePoints,
	const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
	const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
	std::vector<float>& perViewErrors)
{
	std::vector<cv::Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++)
	{
		cv::projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err * err;
		totalPoints += n;
	}
	return std::sqrt(totalErr / totalPoints);
}

void saveCameraParams(
	const cv::Size& imageSize,
	const cv::Mat& cameraMatrix,
	const cv::Mat& distCoeffs,
	const std::string& outFile)
{
	// create camera parameters datastructure
	CameraParameters camParams;
	camParams.resolution.width = imageSize.width;
	camParams.resolution.height = imageSize.height;
	cv::cv2eigen(cameraMatrix, camParams.intrinsic);
	cv::cv2eigen(distCoeffs, camParams.distortion);
	camParams.id = 0;
	camParams.name = "Camera";
	camParams.type = CameraType::RGB;
	// save camera calibration parameters to file
	saveToFile(camParams, outFile);
}
