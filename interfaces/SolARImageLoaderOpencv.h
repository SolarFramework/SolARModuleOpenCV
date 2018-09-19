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

#ifndef SOLARIMAGELOADEROPENCV_H
#define SOLARIMAGELOADEROPENCV_H

#include "api/image/IImageLoader.h"

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include <string>


namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARImageLoaderOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::image::IImageLoader {
public:
    SolARImageLoaderOpencv();
    ~SolARImageLoaderOpencv();
    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

    void unloadComponent () override final;
    ///
    /// \brief getImage method returns the image previously loaded when its configuration parameter path has been set
    ///
    FrameworkReturnCode getImage(SRef<Image> & img);

    ///
    /// \brief reloadImage method load a image if as instance its path (set as a configuration parameter of the implemented component) has changed
    ///
    FrameworkReturnCode reloadImage();

private:
    /// @brief The path of the image to load
    std::string m_filePath = "";

    SRef<Image> m_img;
};

}
}
}  // end of namespace SolAR



#endif
