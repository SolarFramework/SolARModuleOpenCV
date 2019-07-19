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
#include "SolARBaseCameraOpencv.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARCameraOpencv
 * @brief <B>Grabs current image captured by a RGB camera.</B>
 * <TT>UUID: 5b7396f4-a804-4f3c-a0eb-fb1d56042bb4</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARCameraOpencv : public SolARBaseCameraOpencv {
public:
    SolARCameraOpencv(); // to replace with ISolARDeviceInfo ! should be set later with init method ? default behavior on devices with facefront/rear embedded cams ?

    ~SolARCameraOpencv() override;

    /// @brief Start the acquisition device referenced by its device_id
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode start() override;

    /// @brief Fill the SRef img buffer with a last image captured by the camera device.
    /// @return FrameworkReturnCode to track sucessful or failing event.
    FrameworkReturnCode getNextImage(SRef<Image> & img) override;
    //params getCameraIntrinsics() override;
    //Frame : image + timestamp image + depth + timestamp depth ...
    void unloadComponent () override final;

 private:

     /// @brief The ID of the camera to capture with
     uint32_t m_deviceID;
};

}
}
}

#endif
