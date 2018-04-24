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
#include "ComponentBase.h"
#include "SolAROpencvAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARPerspectiveControllerOpencv : public org::bcom::xpcf::ComponentBase,
        public api::image::IPerspectiveController {
public:
    SolARPerspectiveControllerOpencv();
    ~SolARPerspectiveControllerOpencv() = default;

    void setParameters (const Sizei outputImageSize)  override;

    FrameworkReturnCode correct(const SRef<Image> inputImg, std::vector<SRef<Contour2Df>> & contours, std::vector<SRef<Image>> & patches) override;
    FrameworkReturnCode correct(const SRef<Image> inputImg, SRef<Contour2Df> & contour, SRef<Image> & patch) override;

    void unloadComponent () override final;

private:
    Sizei m_outputImageSize;
};

}
}
}  // end of namespace Solar

#endif // SOLARPERSPECTIVECONTROLLEROPENCV_H
