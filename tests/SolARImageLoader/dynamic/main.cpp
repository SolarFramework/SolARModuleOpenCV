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

#include <iostream>
#include <boost/log/core.hpp>
#include <string>

// ADD COMPONENTS HEADERS HERE, e.g #include "SolarComponent.h"

#include "xpcf/xpcf.h"
#include "SolARModuleOpencv_traits.h"
#include "api/image/IImageLoader.h"
#include "api/display/IImageViewer.h"
#include "core/Log.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int main(int argc, char *argv[])
{

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("conf_ImageLoader.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_ImageLoader.xml")
        return -1;
    }

    // declare and create components
    LOG_INFO("Start creating components");

    SRef<image::IImageLoader> imageLoader = xpcfComponentManager->create<SolAR::MODULES::OPENCV::SolARImageLoaderOpencv>()->bindTo<image::IImageLoader>();
    SRef<display::IImageViewer> viewerConfImage = xpcfComponentManager->create<SolAR::MODULES::OPENCV::SolARImageViewerOpencv>("confImage")->bindTo<display::IImageViewer>();
    SRef<display::IImageViewer> viewerParamImage = xpcfComponentManager->create<SolAR::MODULES::OPENCV::SolARImageViewerOpencv>("paramImage")->bindTo<display::IImageViewer>();

    if (!imageLoader || !viewerConfImage || !viewerParamImage)
    {
        LOG_ERROR("One or more component creations have failed");
        return -1;
    }

    // Data structure declaration
    SRef<Image> imageConf;
    SRef<Image> imageParam;

    // Get image according to the path defined in the configuration file conf_ImageLoader.xml
    if (imageLoader->getImage(imageConf) != FrameworkReturnCode::_SUCCESS)
    {
        LOG_ERROR("Cannot load image from configuration file with path {}", imageLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
        return -1;
    }

    // load and get image according the path given in argument of this executable
    if (argc == 2)
    {
        // Change the path of the image according to the argument given in parameter
        imageLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->setStringValue(argv[1],0);
        imageLoader->reloadImage();
        if (imageLoader->getImage(imageParam) != FrameworkReturnCode::_SUCCESS)
        {
           LOG_ERROR("Cannot load image from parameters with path {}", imageLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
           return -1;
        }
    }

    // Display images in dedicated windows
    while (true)
    {
        if (argc > 1)
        {
            if (viewerConfImage->display(imageConf) == FrameworkReturnCode::_STOP || viewerParamImage->display(imageParam) == FrameworkReturnCode::_STOP )
            {
                LOG_INFO("end of SolARImageopenCV test");
                break;
            }
        }
        else
        {
            if (viewerConfImage->display(imageConf) == FrameworkReturnCode::_STOP)
            {
                LOG_INFO("end of SolARImageopenCV test");
                break;
            }
        }
    }

    return 0;
}
