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

#ifndef SOLARIMAGEVIEWERDURATIONOPENCV_H
#define SOLARIMAGEVIEWERDURATIONOPENCV_H

#include "api/display/IImageViewer.h"

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include <string>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARImageViewerDurationOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::display::IImageViewer {
public:
    SolARImageViewerDurationOpencv();
    ~SolARImageViewerDurationOpencv();
    void unloadComponent () override final;

    /// \brief this method displays an image contained in a Image object in a window
    /// @param[in] img The image to display in the window
    /// @return FrameworkReturnCode::_SUCCESS if the window is created, else FrameworkReturnCode::_ERROR_ or if the duration has passed.
    FrameworkReturnCode display(SRef<Image> img) override;

private:
    /// @brief the title of the window on which the image will be displayed
    std::string m_title = "";

    /// @brief the width of the window on which the image will be displayed (if <=0, the width of the input image)
    int m_width = 0;

    /// @brief the height of the window on which the image will be displayed (if <=0, the height of the input image)
    int m_height = 0;

    /// @brief The duration in milliseconds before closing the window.
    unsigned int m_duration;

};

}
}
}  // end of namespace SolAR

#endif
