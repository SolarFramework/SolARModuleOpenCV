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
#include "opencv2/opencv.hpp"

#include "ComponentBase.h"
#include "api/input/devices/ICamera.h"

#include "SolAROpencvAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARCameraOpencv : public org::bcom::xpcf::ComponentBase,
        public api::input::devices::ICamera {
public:
    SolARCameraOpencv(); // to replace with ISolARDeviceInfo ! should be set later with init method ? default behavior on devices with facefront/rear embedded cams ?

    ~SolARCameraOpencv() = default;

    FrameworkReturnCode start(uint32_t device_id) override;
    FrameworkReturnCode start(std::string inputFileName)override;

    FrameworkReturnCode loadCameraParameters (const std::string & filename) override;

    FrameworkReturnCode getNextImage(SRef<Image> & img) override;

    void setResolution(Sizei resolution) override;
    void setIntrinsicParameters(const CamCalibration & intrinsic_parameters) override;
    void setDistorsionParameters(const CamDistortion & distorsion_parameters) override;

    Sizei getResolution () override;
    CamCalibration getIntrinsicsParameters() override;
    CamDistortion getDistorsionParameters() override;

    //params getCameraIntrinsics() override;
    //Frame : image + timestamp image + depth + timestamp depth ...
    void unloadComponent () override final;

 private:
     int m_device_id;
     cv::VideoCapture m_capture;
     Sizei m_resolution;

     CamCalibration m_intrinsic_parameters;
     CamDistortion m_distorsion_parameters;

};

}
}
}

#endif
