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

#ifndef SOLARPERSPECTIVECONTROLLEROPENCV_H
#define SOLARPERSPECTIVECONTROLLEROPENCV_H

#include "api/image/IPerspectiveController.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARPerspectiveControllerOpencv
 * @brief <B>Extracts an unwrapped image from a specific region of an input image defined with four 2D points.</B>
 * <TT>UUID: 9c960f2a-cd6e-11e7-abc4-cec278b6b50a</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ outputImageWidth,
 *                          the width in pixels of the output image,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 640 }}
 * @SolARComponentProperty{ outputImageHeight,
 *                          the Height in pixels of the output image,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 480 }}
 * 
 * @SolARComponentPropertiesEnd
 */

class SOLAROPENCV_EXPORT_API SolARPerspectiveControllerOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::image::IPerspectiveController {
public:
    SolARPerspectiveControllerOpencv();
    ~SolARPerspectiveControllerOpencv() = default;

    FrameworkReturnCode correct(const SRef<datastructure::Image> inputImg, const std::vector<datastructure::Contour2Df> & contours, std::vector<SRef<datastructure::Image>> & patches) override;
    FrameworkReturnCode correct(const SRef<datastructure::Image> inputImg, const datastructure::Contour2Df & contour, SRef<datastructure::Image> & patch) override;

    void unloadComponent () override final;

private:
    /// @brief The width in pixels of the output image
    int m_outputImageWidth = 640;
    /// @brief The Height in pixels of the output image
    int m_outputImageHeight = 480;
};

}
}
}  // end of namespace Solar

#endif // SOLARPERSPECTIVECONTROLLEROPENCV_H
