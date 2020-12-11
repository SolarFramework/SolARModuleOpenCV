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

#ifndef SOLARIMAGESASCAMERAOPENCV_H
#define SOLARIMAGESASCAMERAOPENCV_H

#include <vector>
#include <string>
#include "SolARBaseCameraOpencv.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARImagesAsCameraOpencv
 * @brief <B>Loads an image sequence stored in a dedicated folder.</B>
 * <TT>UUID: b8a8b963-ba55-4ea4-b045-d9e7e8f6db02</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARImagesAsCameraOpencv : public SolARBaseCameraOpencv {
public:
    SolARImagesAsCameraOpencv(); // to replace with ISolARDeviceInfo ! should be set later with init method ? default behavior on devices with facefront/rear embedded cams ?

    ~SolARImagesAsCameraOpencv() override = default;

    /// @brief Start the Images acquisition
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode start() override;

    FrameworkReturnCode getNextImage(SRef<datastructure::Image> & img) override;

    //params getCameraIntrinsics() override;
    //Frame : image + timestamp image + depth + timestamp depth ...
    void unloadComponent () override final;

private:
    // @brief Path to the images which will be used as a camera capture
    std::string m_ImagesDirectoryPath = "";
    std::vector<std::string> imagePaths;           //will contained the path of the images to us
	// @brief time delay camera between two images
	int m_delayTime = 30;

};

}
}
}

#endif // SOLARIMAGESASCAMERAOPENCV_H
