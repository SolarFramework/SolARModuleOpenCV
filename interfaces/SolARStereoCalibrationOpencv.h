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

#ifndef SOLARSTEREOCALIBRATIONOPENCV_H
#define SOLARSTEREOCALIBRATIONOPENCV_H

#include "api/stereo/IStereoCalibration.h"
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "xpcf/component/ComponentBase.h"
#include "SolAROpencvAPI.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARStereoCalibrationOpencv
 * @brief <B>Calibrate and rectify a stereo camera.</B>
 * <TT>UUID: 31051575-1521-4559-9e75-e7e97f990c77</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARStereoCalibrationOpencv :
	public org::bcom::xpcf::ComponentBase,
	public api::stereo::IStereoCalibration
{
public:
	/// @brief SolARStereoCalibrationOpencv constructor
	SolARStereoCalibrationOpencv();

	/// @brief SolARStereoCalibrationOpencv destructor
    ~SolARStereoCalibrationOpencv() override;

	/// @brief Calibrate a stereo camera from a set of captured images and output the result in the given file
	/// @param[in] images1 Set of images from the first camera
	/// @param[in] images2 Set of images from the second camera
	/// @param[in] calibrationFilePath file path
	/// @return FrameworkReturnCode::_SUCCESS if calibration succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode calibrate(const std::vector<SRef<SolAR::datastructure::Image>>& images1,
								const std::vector<SRef<SolAR::datastructure::Image>>& images2,
								const std::string & calibrationFilePath) override;

	/// @brief Computes rectification transforms of a calibrated stereo camera and output the result in the given file
	/// @param[in] calibrationFilePath calibration file path
	/// @param[in] rectificationFilePath rectification file path
	/// @return FrameworkReturnCode::_SUCCESS if rectification succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode rectify(const std::string & calibrationFilePath,
								const std::string & rectificationFilePath) override;

	/// @brief this method is used to set camera parameters for the first camera
	/// @param[in] camParams camera parameters of the first camera
	void setCameraParameters1(const SolAR::datastructure::CameraParameters & camParams) override;

	/// @brief this method is used to set camera parameters for the second camera
	/// @param[in] camParams camera parameters of the second camera
	void setCameraParameters2(const SolAR::datastructure::CameraParameters & camParams) override;

	/// @brief Set configuration of stereo calibration
	/// @param[in] configFile configuration file
	void setConfiguration(const std::string & configFile) override;
    
	void unloadComponent() override;

private:
	/// @brief Find chessboard corners in stereo image
	bool findChessboardCornersImage(SRef<datastructure::Image>& image1, 
									SRef<datastructure::Image> image2,
									cv::Mat& displayImage1, 
									cv::Mat& displayImage2, 
									std::vector<cv::Point2f>& corners1,
									std::vector<cv::Point2f>& corners2);

private:
	bool m_isSetConfig = false;
	bool m_isSetParams1 = false;
	bool m_isSetParams2 = false;
	cv::Mat m_intrinsic1, m_intrinsic2, m_distortion1, m_distortion2;
	cv::Size m_boardSize;
	cv::Size m_imageSize;
	float m_squareSize;
	int m_nbFrames;
	int m_nbDropFrames;
	int m_waitTime;
};
}
}
}  // end of namespace Solar
#endif // SOLARSTEREOCALIBRATIONOPENCV_H