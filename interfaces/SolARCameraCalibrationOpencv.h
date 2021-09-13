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
#include "xpcf/component/ConfigurableBase.h"

#include "SolAROpencvAPI.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARCameraCalibrationOpencv
 * @brief <B>Calibrates a camera based on a chessboard.</B>
 * <TT>UUID: 702a7f53-e5ec-45d2-887d-daa99a34a33c</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARCameraCalibrationOpencv :
	public org::bcom::xpcf::ConfigurableBase,
	public api::input::devices::ICameraCalibration
{
public:
    SolARCameraCalibrationOpencv();
    ~SolARCameraCalibrationOpencv() override;
    /// @brief Calibrate the camera device from a sequence of images
    /// @param[in] images The set of images for calibration
    /// @param[out] camParams The camera paramters
    /// @return FrameworkReturnCode::_SUCCESS if calibration succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode calibrate(const std::vector<SRef<SolAR::datastructure::Image>>& images,
                                  SolAR::datastructure::CameraParameters & camParams) override;

	void unloadComponent() override;

private:
	bool findChessboardCornersImage(SRef<SolAR::datastructure::Image>& image,
									cv::Mat & displayImage,
									std::vector<cv::Point2f>& corners);

private:
    cv::Mat m_camMatrix;
    cv::Mat m_camDistortion;
	cv::Size m_boardSize;
	cv::Size m_imageSize;	
	float m_squareSize;
	int m_nbFrames = 30;
	int m_nbDropFrames = 0;
	int m_flags;
	int m_waitTime = 30;	
};

}
}
}  // end of namespace Solar
#endif // SOLARCAMERACALIBRATIONOPENCV_H
