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
#include <chrono>

// ADD COMPONENTS HEADERS HERE, e.g #include "SolarComponent.h"

#include "xpcf/xpcf.h"
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

    try {
        /* instantiate component manager*/
        /* this is needed in dynamic mode */
        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if(xpcfComponentManager->load("SolARTest_ModuleOpenCV_ImageCompression_conf.xml")!=org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file SolARTest_ModuleOpenCV_ImageCompression_conf.xml")
            return -1;
        }

        uint8_t encodingQuality = 100;
        if (argc > 1)
        {
            encodingQuality = (uint8_t)atoi(argv[1]);
        }
        // declare and create components
        LOG_INFO("Start creating components");

        SRef<image::IImageLoader> imageLoader = xpcfComponentManager->resolve<image::IImageLoader>();
        SRef<display::IImageViewer> viewerJPEGImage = xpcfComponentManager->resolve<display::IImageViewer>("imageJPEG");
        SRef<display::IImageViewer> viewerPNGImage = xpcfComponentManager->resolve<display::IImageViewer>("imagePNG");

        if (!imageLoader || !viewerJPEGImage || !viewerPNGImage)
        {
            LOG_ERROR("One or more component creations have failed");
            return -1;
        }

        // Data structure declaration
        SRef<Image> inputImageSRef;
        SRef<Image> JPEGImageSRef;
        SRef<Image> PNGImageSRef;
        std::string jpeg_filename = "jpegImage.archive";
        std::string png_filename = "pngImage.archive";

        // Get image according to the path defined in the configuration file conf_ImageLoader.xml
        if (imageLoader->getImage(inputImageSRef) != FrameworkReturnCode::_SUCCESS)
        {
            LOG_ERROR("Cannot load image from configuration file with path {}", imageLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->getStringValue());
            return -1;
        }

        // JPEG COMPRESSION
		inputImageSRef->setImageEncoding(Image::ENCODING_JPEG); 
        inputImageSRef->setImageEncodingQuality(encodingQuality);
        LOG_INFO("JPEG Encoding quality: {}", encodingQuality);
        std::ofstream out_jpeg(jpeg_filename, std::ios_base::out | std::ios_base::binary);
        boost::archive::binary_oarchive output_archive_JPEG(out_jpeg);
        auto start = std::chrono::high_resolution_clock::now();
        output_archive_JPEG & BOOST_SERIALIZATION_NVP(*inputImageSRef);
        auto end = std::chrono::high_resolution_clock::now();
        LOG_INFO("JPEG compression time :{}ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        out_jpeg.close();

        std::ifstream in_jpeg(jpeg_filename, std::ios_base::in | std::ios_base::binary);
        boost::archive::binary_iarchive input_archive_JPEG(in_jpeg);
        Image JPEGImage;
        start = std::chrono::high_resolution_clock::now();
        input_archive_JPEG & BOOST_SERIALIZATION_NVP(JPEGImage);
        end = std::chrono::high_resolution_clock::now();
        LOG_INFO("JPEG decompression time :{}ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        JPEGImageSRef = JPEGImage.copy();
        in_jpeg.close();

		// PNG Compression
		inputImageSRef->setImageEncoding(Image::ENCODING_PNG); 
        inputImageSRef->setImageEncodingQuality(encodingQuality);
        LOG_INFO("PNG Encoding quality: {}", encodingQuality);
        std::ofstream out_png(png_filename, std::ios_base::out | std::ios_base::binary);
        boost::archive::binary_oarchive output_archive_PNG(out_png);
        start = std::chrono::high_resolution_clock::now();
        output_archive_PNG & BOOST_SERIALIZATION_NVP(*inputImageSRef);
        end = std::chrono::high_resolution_clock::now();
		LOG_INFO("PNG compression time :{}ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        out_png.close();


        std::ifstream in_png(png_filename, std::ios_base::in | std::ios_base::binary);
        boost::archive::binary_iarchive input_archive_PNG(in_png);
        Image PNGImage;
        start = std::chrono::high_resolution_clock::now();
        input_archive_PNG & BOOST_SERIALIZATION_NVP(PNGImage);
		end = std::chrono::high_resolution_clock::now();
		LOG_INFO("PNG decompression time :{}ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        PNGImageSRef = PNGImage.copy();
        in_png.close();

        // Display images in dedicated windows
        while (true)
        {
            if (viewerJPEGImage->display(JPEGImageSRef) == FrameworkReturnCode::_STOP || viewerPNGImage->display(PNGImageSRef) == FrameworkReturnCode::_STOP)
            {
                LOG_INFO("end of SolARImageopenCV test");
                break;
            }
        }
    }

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }
    return 0;
}
