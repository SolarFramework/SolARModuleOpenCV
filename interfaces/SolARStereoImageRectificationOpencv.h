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

#ifndef SOLARSTEREOIMAGERECTIFICATIONOPENCV_H
#define SOLARSTEREOIMAGERECTIFICATIONOPENCV_H

#include "api/image/IImageRectification.h"
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARStereoImageRectificationOpencv
* @brief <B>Rectify image.</B>
* <TT>UUID: 427cbbb8-6afe-4b3b-8b04-cb93ed925b40</TT>
*
*/

class SOLAROPENCV_EXPORT_API SolARStereoImageRectificationOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::image::IImageRectification
{
public:
    /// @brief SolARStereoImageRectificationOpencv constructor
    SolARStereoImageRectificationOpencv();

    /// @brief SolARStereoImageRectificationOpencv destructor
    ~SolARStereoImageRectificationOpencv() override;

    /// @brief Rectify image
    /// @param[in] image The input image
    /// @param[in] camParams The camera parameters of camera
    /// @param[in] rectParams The rectification parameters of camera
    /// @param[out] rectifiedImage The rectified image
    /// @return FrameworkReturnCode::_SUCCESS if rectifying succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode rectify(SRef<SolAR::datastructure::Image> image,
                                const SolAR::datastructure::CameraParameters& camParams,
                                const SolAR::datastructure::RectificationParameters& rectParams,
                                SRef<SolAR::datastructure::Image>& rectifiedImage) override;

	void unloadComponent() override;
};
}
}
}  // end of namespace Solar
#endif // SOLARSTEREOIMAGERECTIFICATIONOPENCV_H
