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

#ifndef SOLARCAMERAOPENCV_H
#define SOLARCAMERAOPENCV_H

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
 * @brief The SolARCameraOpencv class
 *
 */
class SOLAROPENCV_EXPORT_API SolARCameraOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::input::devices::ICamera {
public:
    SolARCameraOpencv(); // to replace with ISolARDeviceInfo ! should be set later with init method ? default behavior on devices with facefront/rear embedded cams ?

    ~SolARCameraOpencv();

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

    /// @brief Start the acquisition device referenced by its device_id
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode start() override;

    /// @brief Stop the acquisition device
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode stop() override;

    /// @brief Fill the SRef img buffer with a last image captured by the camera device.
    /// @return FrameworkReturnCode to track sucessful or failing event.
    FrameworkReturnCode getNextImage(SRef<Image> & img) override;
    /// @brief Set the size of the grabbed image from the camera.
    ///[in] resolution: the width and height of the output grabbed image.
    void setResolution(Sizei resolution) override;
    /// @brief Set the camera intrinsic parameters from a calibration matrix.
    ///[in] intrinsic_parameters: Calibration matrix containing the nine camera calibration parameters.
    void setIntrinsicParameters(const CamCalibration & intrinsic_parameters) override;
    /// @brief Set the camera distorsion parameters from a distorsion matrix.
    ///[in] distorsion_parameters: Distorsion matrix containing the five camera distorsion parameters.
    void setDistorsionParameters(const CamDistortion & distorsion_parameters) override;
    /// @brief Get the current resolutio nof the camera.
    ///[out]  width and height of the images grabbed from the camera.
    Sizei getResolution () override;
    /// @brief Get the current camera calibration parameters.
    ///[out] Calibration matrix containing the nine camera calibration parameters.
    CamCalibration getIntrinsicsParameters() override;
    /// @brief Get the current camera distorsion parameters.
    ///[out] Distorsion matrix containing the five camera distorsion parameters.
    CamDistortion getDistorsionParameters() override;
    //params getCameraIntrinsics() override;
    //Frame : image + timestamp image + depth + timestamp depth ...
    void unloadComponent () override final;

 private:
     /// @brief Path to the calibration file of the camera
     std::string m_calibrationFile = "";

     /// @brief The ID of the camera to capture with
     unsigned int m_deviceID;

     cv::VideoCapture m_capture;
     bool m_is_resolution_set;
     Sizei m_resolution;

     CamCalibration m_intrinsic_parameters;
     CamDistortion m_distorsion_parameters;

};

}
}
}

#endif
