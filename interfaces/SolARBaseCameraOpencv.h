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

#ifndef SolARBaseCameraOpencv_H
#define SolARBaseCameraOpencv_H

#include <vector>
#include <string>
#include "api/input/devices/ICamera.h"

#include "opencv2/opencv.hpp"

#include "xpcf/component/ConfigurableBase.h"

#include "SolAROpencvAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARBaseCameraOpencv
 * @brief <B>Grabs current image captured by a RGB camera.</B>
 * <TT>UUID: 5b7396f4-a804-4f3c-a0eb-fb1d56042bb4</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARBaseCameraOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::input::devices::ICamera {
public:
    SolARBaseCameraOpencv(const org::bcom::xpcf::uuids::uuid & uuid); // to replace with ISolARDeviceInfo ! should be set later with init method ? default behavior on devices with facefront/rear embedded cams ?

    ~SolARBaseCameraOpencv() override;

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

    /// @brief Stop the acquisition device
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode stop() override;

    /// @brief Set the size of the grabbed image from the camera.
    ///[in] resolution: the width and height of the output grabbed image.
    void setResolution(const Sizei & resolution) override;
    /// @brief Set the camera intrinsic parameters from a calibration matrix.
    ///[in] intrinsic_parameters: Calibration matrix containing the nine camera calibration parameters.
    void setIntrinsicParameters(const CamCalibration & intrinsic_parameters) override;
    /// @brief Set the camera distortion parameters from a distortion matrix.
    ///[in] distortion_parameters: distortion matrix containing the five camera distortion parameters.
    void setDistortionParameters(const CamDistortion & distortion_parameters) override;
    /// @brief Set the camera parameters
    void setParameters(const CameraParameters & parameters) override;
    /// @return Return the camera parameters
    const CameraParameters & getParameters() override;
    /// @brief Get the current resolutio nof the camera.
    ///[out]  width and height of the images grabbed from the camera.
    Sizei getResolution () override;
    /// @brief Get the current camera calibration parameters.
    ///[out] Calibration matrix containing the nine camera calibration parameters.
    const CamCalibration & getIntrinsicsParameters() override;
    /// @brief Get the current camera distortion parameters.
    ///[out] distortion matrix containing the five camera distortion parameters.
    const CamDistortion & getDistortionParameters() override;
    //params getCameraIntrinsics() override;
    //Frame : image + timestamp image + depth + timestamp depth ...
    void unloadComponent () override;

 protected:
     /// @brief Path to the calibration file of the camera
     std::string m_calibrationFile = "";
     cv::VideoCapture m_capture;
     bool m_is_resolution_set;
     CameraParameters m_parameters;
};

}
}
}

#endif
