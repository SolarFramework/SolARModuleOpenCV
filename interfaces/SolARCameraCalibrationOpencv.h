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

#ifndef SOLARCAMERACALIBRATIONOPENCV_H
#define SOLARCAMERACALIBRATIONOPENCV_H

#include "api/input/devices/ICameraCalibration.h"

#include <string>
#include "opencv2/videoio.hpp"
#include "xpcf/component/ComponentBase.h"

#include "SolAROpencvAPI.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARCameraCalibrationOpencv :
	public org::bcom::xpcf::ComponentBase,
	public api::input::devices::ICameraCalibration
{
public:
	enum ProcessMode
	{
		SOLAR_DETECT = 0,
		SOLAR_CAPTURE = 1,
		SOLAR_CALIBRATED = 2
	};

public:
	SolARCameraCalibrationOpencv();
	virtual ~SolARCameraCalibrationOpencv();

	bool calibrate(std::string&inputVideo, std::string&output);
	bool calibrate(int camera_id, std::string&output);
	bool setParameters(std::string&config_file);
	virtual void unloadComponent() override;


private:
protected:
	cv::Size m_boardSize;
	cv::Size m_imageSize;
	cv::Mat m_camMatrix;
	cv::Mat m_camDistorsion;

	float m_squareSize;
	float m_aspectRatio;

	int m_nframes;
	int m_flags;
	int m_delay;

	virtual bool process(cv::VideoCapture&, std::string&);
	
	static double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
		std::vector<float>& perViewErrors);

	static void calcChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners);

	static bool runCalibration(std::vector<std::vector<cv::Point2f>>imagePoints,
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
		double& totalAvgErr);

	static void saveCameraParams(const std::string& filename,
		cv::Size imageSize,
		cv::Size boardSize,
		float squareSize,
		float aspectRatio,
		int flags,
		const cv::Mat& cameraMatrix,
		const cv::Mat& distCoeffs);

	static bool runAndSave(const std::string& outputFilename,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		cv::Size imageSize,
		cv::Size boardSize,
		float squareSize,
		float aspectRatio,
		int flags,
		cv::Mat& cameraMatrix,
		cv::Mat& distCoeffs);
};
}
}
}  // end of namespace Solar
#endif // SOLARCAMERACALIBRATIONOPENCV_H
