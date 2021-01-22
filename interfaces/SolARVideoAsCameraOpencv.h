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
#include "SolARBaseCameraOpencv.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARVideoAsCameraOpencv
* @brief <B>Grabs the images from a video file.</B>
* <TT>UUID: fa4a780a-9720-11e8-9eb6-529269fb1459</TT>
*
* @SolARComponentPropertiesBegin
* @SolARComponentProperty{ videoPath,
*                          Path to the video file which will be streamed as a camera capture,
*                          @SolARComponentPropertyDescString{ "" }}
* @SolARComponentProperty{ delayTime,
*                          time delay camera between two images,
*                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 30 }}
* @SolARComponentPropertiesEnd
* 
*/

class SOLAROPENCV_EXPORT_API SolARVideoAsCameraOpencv : public SolARBaseCameraOpencv {
public:
    SolARVideoAsCameraOpencv(); // to replace with ISolARDeviceInfo ! should be set later with init method ? default behavior on devices with facefront/rear embedded cams ?

    ~SolARVideoAsCameraOpencv() override = default;

    /// @brief Start the video acquisition
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode start() override;

    FrameworkReturnCode getNextImage(SRef<datastructure::Image> & img) override;

    void unloadComponent () override final;

private:
    // @brief Path to the video file which will be streamed as a camera capture
    std::string m_videoPath = "";

	// @brief time delay camera between two images
	int m_delayTime = 30;

};

}
}
}

#endif // SOLARVIDEOASCAMERAOPENCV_H
