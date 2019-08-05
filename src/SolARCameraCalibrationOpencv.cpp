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
#include "core/Log.h"

// Opencv dependencies
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARCameraCalibrationOpencv)


namespace SolAR {
namespace MODULES {
namespace OPENCV {



SolARCameraCalibrationOpencv::SolARCameraCalibrationOpencv():ComponentBase(xpcf::toUUID<SolARCameraCalibrationOpencv>())
{
    declareInterface<api::input::devices::ICameraCalibration>(this);
}


SolARCameraCalibrationOpencv::~SolARCameraCalibrationOpencv()
{
    LOG_DEBUG("SolARCameraCalibrationOpencv destructor")
}



double SolARCameraCalibrationOpencv::computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
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
        totalErr += err*err;
        totalPoints += n;
    }
    return std::sqrt(totalErr / totalPoints);
}

void SolARCameraCalibrationOpencv::calcChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners)
{
    corners.resize(0);
    corners.reserve(boardSize.height * boardSize.width);
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            corners.emplace_back(j*squareSize, i*squareSize, 0);
}

bool SolARCameraCalibrationOpencv::runCalibration(const std::vector<std::vector<cv::Point2f>> & imagePoints,
												cv::Size imageSize,
												cv::Size boardSize,
												float squareSize,
												float aspectRatio,
												int flags,
												cv::Mat& cameraMatrix,
												cv::Mat& distCoeffs,
												std::vector<cv::Mat>& rvecs,
												std::vector<cv::Mat>& tvecs,
												std::vector<float>& reprojErrs,
												double& totalAvgErr)
{
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    if ((flags & cv::CALIB_FIX_ASPECT_RATIO) != 0)
        cameraMatrix.at<double>(0, 0) = aspectRatio;

    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    std::vector<std::vector<cv::Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0]);

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
        distCoeffs, rvecs, tvecs, flags | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
    ///*|CALIB_FIX_K3*/|CALIB_FIX_K4|CALIB_FIX_K5);
    LOG_INFO("RMS error reported by calibrateCamera: {:g}", rms)

    bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
        rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

void SolARCameraCalibrationOpencv::saveCameraParams(const std::string& filename,
													cv::Size imageSize,
													cv::Size boardSize,
													float squareSize,
													float aspectRatio,
													int flags,
													const cv::Mat& cameraMatrix,
													const cv::Mat& distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if (flags & cv::CALIB_FIX_ASPECT_RATIO)
        fs << "aspectRatio" << aspectRatio;

    if (flags != 0)
    {
        sprintf(buf, "flags: %s%s%s%s",
            flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
        //cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

}


bool SolARCameraCalibrationOpencv::runAndSave(const std::string & outputFilename,
											const std::vector<std::vector<cv::Point2f> >& imagePoints,
											cv::Size imageSize,
											cv::Size boardSize,
											float squareSize,
											float aspectRatio,
											int flags,
											cv::Mat& cameraMatrix,
											cv::Mat& distCoeffs)
{
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(imagePoints, imageSize, boardSize, squareSize,
        aspectRatio, flags, cameraMatrix, distCoeffs,
        rvecs, tvecs, reprojErrs, totalAvgErr);
    LOG_INFO("{} avg reprojection error = {:.2f}", ok ? "Calibration succeeded" : "Calibration failed", totalAvgErr)


    if (ok)
        saveCameraParams(outputFilename, imageSize,
            boardSize, squareSize, aspectRatio,
            flags, cameraMatrix, distCoeffs);
    return ok;
}

bool SolARCameraCalibrationOpencv::calibrate(const std::string & inputVideo, const std::string & output)
{
	cv::VideoCapture capture;

	if (!capture.open(inputVideo)) // videoFile
	{
		LOG_ERROR("Video with url {} does not exist", inputVideo);
		return false;
	}

	return process(capture, output);
}

bool SolARCameraCalibrationOpencv::calibrate(int camera_id, const std::string & output)
{
	cv::VideoCapture capture;

	if (!capture.open(camera_id)) // camera id
	{
		LOG_ERROR("cannot open camera id #{} ", camera_id);
		return false;
	}

	return process(capture, output);
}


bool SolARCameraCalibrationOpencv::process(cv::VideoCapture & capture, const std::string & output)
{
	cv::Size imageSize;
	int i;
	clock_t prevTimestamp = 0;
	ProcessMode mode = SOLAR_DETECT;
	std::vector<std::vector<cv::Point2f> > imagePoints;

    for (i = 0;; i++){
        cv::Mat view, viewGray;
        bool blink = false;

        if (capture.isOpened())
        {
            cv::Mat view0;
            capture >> view0;
            view0.copyTo(view);
        }
        imageSize = view.size();

        std::vector<cv::Point2f> pointbuf;
        cv::cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
        bool found;
            found = cv::findChessboardCorners(view, m_boardSize, pointbuf,
                cv::CALIB_CB_ADAPTIVE_THRESH);

        // improve the found corners' coordinate accuracy
        if (found) cornerSubPix(viewGray, pointbuf, cv::Size(11, 11),
            cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

        if (mode == SOLAR_CAPTURE && found &&
            (!capture.isOpened() || clock() - prevTimestamp > m_delay*1e-3*CLOCKS_PER_SEC)){
            imagePoints.emplace_back(pointbuf);
            prevTimestamp = clock();
            blink = capture.isOpened();
        }

        if (found)
            cv::drawChessboardCorners(view, m_boardSize, cv::Mat(pointbuf), found);

        std::string msg = mode == SOLAR_CAPTURE ? "100/100" :
            mode == SOLAR_CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;

        cv::Size textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
        cv::Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

        if (mode == SOLAR_CAPTURE)
                msg = cv::format("%d/%d", (int)imagePoints.size(), m_nframes);

        cv::putText(view, msg, textOrigin, 1, 1,
            mode != SOLAR_CALIBRATED ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));

        if (blink)
            cv::bitwise_not(view, view);

        cv::imshow("Image View", view);
        char key = (char)cv::waitKey(capture.isOpened() ? 50 : 500);

        if (key == 27)
            break;

        if (capture.isOpened() && key == 'g'){
            mode = SOLAR_CAPTURE;
            imagePoints.clear();
        }
        if (mode == SOLAR_CAPTURE && imagePoints.size() >= (unsigned)m_nframes){
            if (runAndSave(output, imagePoints, imageSize,
                m_boardSize, m_squareSize, m_aspectRatio,
                m_flags, m_camMatrix, m_camDistorsion))
                mode = SOLAR_CALIBRATED;
            else
                mode = SOLAR_DETECT;
            if (!capture.isOpened())
                break;
        }
    }
    return true;
}

//
//bool SolARCameraCalibrationOpencv::calibrate(int camera_id, std::string&output) {
//	cv::Size imageSize;
//	int i;
//	cv::VideoCapture capture;
//	clock_t prevTimestamp = 0;
//	int mode = DETECTION;
//	std::vector<std::vector<cv::Point2f> > imagePoints;
//
//
//	capture.open(camera_id);
//	for (i = 0;; i++) {
//		cv::Mat view, viewGray;
//		bool blink = false;
//
//		if (capture.isOpened())
//		{
//			cv::Mat view0;
//			capture >> view0;
//			view0.copyTo(view);
//		}
//		imageSize = view.size();
//
//		std::vector<cv::Point2f> pointbuf;
//		cv::cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
//		bool found;
//		found = cv::findChessboardCorners(view, m_boardSize, pointbuf,
//			cv::CALIB_CB_ADAPTIVE_THRESH);
//
//		// improve the found corners' coordinate accuracy
//		if (found) cornerSubPix(viewGray, pointbuf, cv::Size(11, 11),
//			cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
//
//		if (mode == CAPTURING && found &&
//			(!capture.isOpened() || clock() - prevTimestamp > m_delay*1e-3*CLOCKS_PER_SEC)) {
//			imagePoints.emplace_back(pointbuf);
//			prevTimestamp = clock();
//			blink = capture.isOpened();
//		}
//
//		if (found)
//			cv::drawChessboardCorners(view, m_boardSize, cv::Mat(pointbuf), found);
//
//		std::string msg = mode == CAPTURING ? "100/100" :
//			mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
//		int baseLine = 0;
//
//		cv::Size textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
//		cv::Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);
//
//		if (mode == CAPTURING)
//			msg = cv::format("%d/%d", (int)imagePoints.size(), m_nframes);
//
//		cv::putText(view, msg, textOrigin, 1, 1,
//			mode != CALIBRATED ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));
//
//		if (blink)
//			cv::bitwise_not(view, view);
//
//		cv::imshow("Image View", view);
//		char key = (char)cv::waitKey(capture.isOpened() ? 50 : 500);
//
//		if (key == 27)
//			break;
//
//		if (capture.isOpened() && key == 'g') {
//			mode = CAPTURING;
//			imagePoints.clear();
//		}
//		if (mode == CAPTURING && imagePoints.size() >= (unsigned)m_nframes) {
//			if (runAndSave(output, imagePoints, imageSize,
//				m_boardSize, m_squareSize, m_aspectRatio,
//				m_flags, m_camMatrix, m_camDistorsion))
//				mode = CALIBRATED;
//			else
//				mode = DETECTION;
//			if (!capture.isOpened())
//				break;
//		}
//	}
//	return true;
//}

bool SolARCameraCalibrationOpencv::setParameters(const std::string & config_file)
{
    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    m_camMatrix.create(3, 3, CV_32FC1);
    m_camDistorsion.create(4, 1, CV_32FC1);

    if (fs.isOpened())
    {
        m_boardSize.width = (int)fs["chessboard_width"];
        m_boardSize.height = (int)fs["chessboard_height"];
        m_squareSize = (float)fs["square_size"];
        m_aspectRatio = (float)fs["aspect_ratio"];
        m_nframes = (int)fs["nb_frames"];
        m_flags = (int)fs["flags"];
        m_delay = (int)fs["delay"];


        fs.release();

        LOG_DEBUG("<SolARCamera calibration parameters> ")
        LOG_DEBUG("    ->board size: {}", m_boardSize)
        LOG_DEBUG("    ->square size: {}", m_squareSize)
        LOG_DEBUG("    ->aspect ratio: {}", m_aspectRatio)
        LOG_DEBUG("    ->nb frames: {}", m_nframes)
        LOG_DEBUG("    ->flags: {}", m_flags)
        LOG_DEBUG("    ->delay: {}", m_delay)

    }
    else
    {
        LOG_ERROR("Error: SolARCamera calibration config file {} does not exist", config_file)
       return false;
    }
    return true;
}

}
}
}  // end of namespace Solar


