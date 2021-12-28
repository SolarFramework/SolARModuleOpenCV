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

#include "api/input/devices/IStereoCameraCalibration.h"
#include "xpcf/component/ConfigurableBase.h"
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
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

class SOLAROPENCV_EXPORT_API SolARStereoCalibrationOpencv : public org::bcom::xpcf::ConfigurableBase,
	public api::input::devices::IStereoCameraCalibration
{
public:
	/// @brief SolARStereoCalibrationOpencv constructor
	SolARStereoCalibrationOpencv();

	/// @brief SolARStereoCalibrationOpencv destructor
    ~SolARStereoCalibrationOpencv() override;

    /// @brief Calibrate a stereo camera from a set of captured images and output the result in the given file
    /// @param[in] images1 Set of images from the first camera
    /// @param[in] images2 Set of images from the second camera
    /// @param[in] camParams1 Camera parameters of the first camera
    /// @param[in] camParams2 Camera parameters of the second camera
    /// @param[out] transformation Transformation matrix from the frist camera to the second camera
    /// @param[out] rectParams1 Rectification parameters of the first camera
    /// @param[out] rectParams2 Rectification parameters of the second camera
    /// @return FrameworkReturnCode::_SUCCESS if calibration succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode calibrate(const std::vector<SRef<SolAR::datastructure::Image>>& images1,
                                  const std::vector<SRef<SolAR::datastructure::Image>>& images2,
                                  const SolAR::datastructure::CameraParameters & camParams1,
                                  const SolAR::datastructure::CameraParameters & camParams2,
                                  SolAR::datastructure::Transform3Df & transformation,
                                  SolAR::datastructure::RectificationParameters & rectParams1,
                                  SolAR::datastructure::RectificationParameters & rectParams2) override;
    
	void unloadComponent() override;

private:
	/// @brief Find chessboard corners in stereo image
	bool findChessboardCornersImage(SRef<datastructure::Image>& image, 
									cv::Mat& displayImage, 
									std::vector<cv::Point2f>& corners);

private:
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
