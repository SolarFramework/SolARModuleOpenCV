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

#ifndef SOLARIMAGEVIEWEROPENCV_H
#define SOLARIMAGEVIEWEROPENCV_H

#include "api/display/IImageViewer.h"

#include "xpcf/component/ComponentBase.h"
#include "SolAROpencvAPI.h"
#include <string>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARImageViewerOpencv : public org::bcom::xpcf::ComponentBase,
    public api::display::IImageViewer {
public:
    SolARImageViewerOpencv();
    ~SolARImageViewerOpencv();
    void unloadComponent () override final;
    FrameworkReturnCode display(const char * title, SRef<Image> img, int w_window=0, int h_window=0) override;
    FrameworkReturnCode display(const char * title, SRef<Image> img, const char* exitKey, int w_window=0, int h_window=0) override;
    FrameworkReturnCode display(const char * title, SRef<Image> img, uint32_t duration, int w_window=0, int h_window=0);

};

}
}
}  // end of namespace SolAR



#endif
