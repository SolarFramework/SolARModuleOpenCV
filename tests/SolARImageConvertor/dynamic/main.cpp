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

#include "xpcf/xpcf.h"
#include "SolARModuleOpencv_traits.h"
#include "api/image/IImageLoader.h"
#include "api/image/IImageConvertor.h"
#include "api/display/IImageViewer.h"
#include "core/log.h"

#include <iostream>
#include <boost/log/core.hpp>
#include <map>

using namespace std;
using namespace SolAR;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int main(int argc, char **argv)
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("conf_ImageConvertor.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_ImageConvertor.xml")
        return -1;
    }

    // declare and create components
    LOG_INFO("Start creating components");


    // components declarations and creation
    SRef<image::IImageLoader> imageLoader = xpcfComponentManager->create<SolARImageLoaderOpencv>()->bindTo<image::IImageLoader>();
    SRef<image::IImageConvertor> convertor = xpcfComponentManager->create<SolARImageConvertorOpencv>()->bindTo<image::IImageConvertor>();
    SRef<display::IImageViewer> viewer = xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();

    if (!imageLoader || !convertor || !viewer)
    {
        LOG_ERROR("One or more component creations have failed");
        return -1;
    }

    SRef<Image> image;
    SRef<Image> convertedImage;


    // Start
    // Get the image (the path of this image is defined in the conf_ImageConvertor.xml)
    if (imageLoader->getImage(image) != FrameworkReturnCode::_SUCCESS)
    {
       LOG_WARNING("image {} cannot be loaded", imageLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
       return 0;
    }

    convertor->convert(image, convertedImage, Image::LAYOUT_GREY);

    while (true)
    {
        if (viewer->display(convertedImage) == FrameworkReturnCode::_STOP)
        {
            std::cout << "end of ImageConvertor test" << std::endl;
            break;
        }
    }

    return 0;
}
