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

#ifndef SOLARVIDEOASCAMERAOPENCV_H
#define SOLARVIDEOASCAMERAOPENCV_H

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

class SOLAROPENCV_EXPORT_API SolARVideoAsCameraOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::input::devices::ICamera {
public:
    SolARVideoAsCameraOpencv(); // to replace with ISolARDeviceInfo ! should be set later with init method ? default behavior on devices with facefront/rear embedded cams ?

    ~SolARVideoAsCameraOpencv() = default;

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

    FrameworkReturnCode start() override;

    FrameworkReturnCode getNextImage(SRef<Image> & img) override;

    void setResolution(Sizei resolution) override;
    void setIntrinsicParameters(const CamCalibration & intrinsic_parameters) override;
    void setDistorsionParameters(const CamDistortion & distorsion_parameters) override;

    Sizei getResolution () override;
    const CamCalibration& getIntrinsicsParameters() const override;
    const CamDistortion& getDistorsionParameters() const override;

    //params getCameraIntrinsics() override;
    //Frame : image + timestamp image + depth + timestamp depth ...
    void unloadComponent () override final;

 private:
     /// @brief Path to the calibration file of the camera
     std::string m_calibrationFile = "";

     /// @brief Path to the video file which will be streamed as a camera capture
     std::string m_videoPath = "";

     cv::VideoCapture m_capture;
     bool m_is_resolution_set;
     Sizei m_resolution;

     CamCalibration m_intrinsic_parameters;
     CamDistortion m_distorsion_parameters;

};

}
}
}

#endif // SOLARVIDEOASCAMERAOPENCV_H
