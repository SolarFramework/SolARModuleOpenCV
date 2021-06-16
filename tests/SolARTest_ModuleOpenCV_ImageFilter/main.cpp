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
#include <string>
#include <vector>
#include <boost/log/core.hpp>

 // ADD XPCF HEADERS HERE
#include "xpcf/xpcf.h"
#include "core/Log.h"

// ADD COMPONENTS HEADERS HERE
#include "api/image/IImageLoader.h"
#include "api/image/IImageFilter.h"
#include "api/display/IImageViewer.h"
#include <chrono>

using namespace std;
using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int main(int argc, char **argv)
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    try {

        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if(xpcfComponentManager->load("SolARTest_ModuleOpenCV_ImageFilter_conf.xml")!=org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file SolARTest_ModuleOpenCV_ImageFilter_conf.xml")
            return -1;
        }

        // declare and create components
        LOG_INFO("Start creating components");


        // components declarations and creation
        auto imageLoader = xpcfComponentManager->resolve<image::IImageLoader>();
        auto wallisFilter = xpcfComponentManager->resolve<image::IImageFilter>("wallis");
        auto wallisViewer = xpcfComponentManager->resolve<display::IImageViewer>();
        auto wallisViewerGaussianBlur = xpcfComponentManager->resolve<display::IImageViewer>("gaussianBlur");
        auto wallisViewerNonLocalMeans = xpcfComponentManager->resolve<display::IImageViewer>("nonLocalMeans");

        if (!imageLoader || !wallisFilter || !wallisViewer)
        {
            LOG_ERROR("One or more component creations have failed");
            return -1;
        }

        SRef<Image> image;
        SRef<Image> wallisFilteredImage;
        SRef<Image> wallisFilteredImageGaussianBlur;
        SRef<Image> wallisFilteredImageNonLocalMeans;


        // Start
        // Get the image (the path of this image is defined in the conf_ImageConvertor.xml)
        if (imageLoader->getImage(image) != FrameworkReturnCode::_SUCCESS)
        {
           LOG_WARNING("image {} cannot be loaded", imageLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
           return 0;
        }

        // Process Wallis Filter without denoising method
        auto start = std::chrono::steady_clock::now();
        wallisFilter->filter(image, wallisFilteredImage);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        LOG_INFO("Time to process Wallis Filter without denoising: {}s", elapsed_seconds.count());

        // Process Wallis Filter with Gaussian Blur
        wallisFilter->bindTo<xpcf::IConfigurable>()->getProperty("denoisingMethod")->setStringValue("GaussianBlur");
        start = std::chrono::steady_clock::now();
        wallisFilter->filter(image, wallisFilteredImageGaussianBlur);
        end = std::chrono::steady_clock::now();
        elapsed_seconds = end-start;
        LOG_INFO("Time to process Wallis Filter with Gaussian Blur denoising: {}s", elapsed_seconds.count());


        // Process Wallis Filter with Gaussian Blur
        wallisFilter->bindTo<xpcf::IConfigurable>()->getProperty("denoisingMethod")->setStringValue("NonLocalMeans");
        start = std::chrono::steady_clock::now();
        wallisFilter->filter(image, wallisFilteredImageNonLocalMeans);
        end = std::chrono::steady_clock::now();
        elapsed_seconds = end-start;
        LOG_INFO("Time to process Wallis Filter with Non Local Means denoising: {}s", elapsed_seconds.count());

        while (true)
        {
            if (wallisViewer->display(wallisFilteredImage) == FrameworkReturnCode::_STOP
             || wallisViewerGaussianBlur->display(wallisFilteredImageGaussianBlur) == FrameworkReturnCode::_STOP
             || wallisViewerNonLocalMeans->display(wallisFilteredImageNonLocalMeans) == FrameworkReturnCode::_STOP       )
            {
                std::cout << "end of ImageFilter test" << std::endl;
                break;
            }
        }
    }

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catched: {}", e.what());
        return -1;
    }

    return 0;
}
