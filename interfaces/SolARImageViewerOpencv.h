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

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include <string>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARImageViewerOpencv
 * @brief <B>Displays an image in a dedicated window.</B>
 * <TT>UUID: 19ea4e13-7085-4e3f-92ca-93f200ffb01b</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARImageViewerOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::display::IImageViewer {
public:
    SolARImageViewerOpencv();
    ~SolARImageViewerOpencv();
    void unloadComponent () override final;

    /// \brief this method displays an image contained in a Image object in a window
    /// @param[in] img The image to display in the window
    /// @return FrameworkReturnCode::_SUCCESS if the window is created, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode display(const SRef<Image> img) override;

    /// \brief this method displays an image contained in a Image object in a window, get the key ascii code pressed.
    /// @param[in] img The image to display in the window
    /// @param[out] key The pressed key ascii code, 0 if no key is pressed
    /// @return FrameworkReturnCode::_SUCCESS if the window is created, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode displayKey(const SRef<Image> img, char& key) override;

private:
    /// @brief the title of the window on which the image will be displayed
    std::string m_title = "";

    /// @brief the width of the window on which the image will be displayed (if <=0, the width of the input image)
    int m_width = 0;

    /// @brief the height of the window on which the image will be displayed (if <=0, the height of the input image)
    int m_height = 0;

    /// @brief The key code to press to close the window. If negative, no key is defined to close the window
    int m_exitKey = 27;

    /// @brief The duration in milliseconds before closing the window. If negative or null, the window remains open.
    unsigned int m_duration = 0;

    bool m_isFirstDisplay = true;
};

}
}
}  // end of namespace SolAR



#endif
