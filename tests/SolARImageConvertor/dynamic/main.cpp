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

#include <iostream>
#include <map>

using namespace std;
using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int run(int argc, char **argv)
{

    // load library
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    // instantiate module managers

    if(xpcfComponentManager->load("$BCOMDEVROOT/.xpcf/SolAR/xpcf_SolARModuleOpenCV_registry.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("XPCF library load has failed")
        return -1;
    }



    // components declarations and creation
    SRef<image::IImageLoader> imageLoader = xpcfComponentManager->create<SolAR::MODULES::OPENCV::SolARImageLoaderOpencv>()->bindTo<image::IImageLoader>();
    SRef<image::IImageConvertor> convertor = xpcfComponentManager->create<SolAR::MODULES::OPENCV::SolARImageConvertorOpencv>()->bindTo<image::IImageConvertor>();
    SRef<display::IImageViewer> viewer = xpcfComponentManager->create<SolAR::MODULES::OPENCV::SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();

    if (!imageLoader || !convertor || !viewer)
    {
        LOG_ERROR("One or more component creations have failed");
        return -1;
    }

    SRef<Image> image;
    SRef<Image> convertedImage = xpcf::utils::make_shared<Image>(Image::ImageLayout::LAYOUT_GREY,
                                                                 Image::PixelOrder::INTERLEAVED,Image::DataType::TYPE_8U);;

    // The escape key to exit the sample
    char escape_key = 27;

    // USE your components here, e.g SolarComponentInstance->testMethod();
    /*if (imageLoader->loadImage(argv[1], image) != FrameworkReturnCode::_SUCCESS)
    {
       LOG_ERROR("Cannot load image with path {}", argv[1]);
       return -1;
    }*/
    imageLoader->bindTo<xpcf::IConfigurable>()->getProperty("filePath")->setStringValue(argv[1],0);
    if (imageLoader->getImage(image) != FrameworkReturnCode::_SUCCESS)
    {
       LOG_ERROR("Cannot load image with path {}", argv[1]);
       return -1;
    }    

    convertor->convert(image, convertedImage, Image::LAYOUT_GREY);

    bool proceed = true;
    viewer->bindTo<xpcf::IConfigurable>()->getProperty("title")->setStringValue("show image");
    viewer->bindTo<xpcf::IConfigurable>()->getProperty("exitKey")->setIntegerValue(27);    
    while (proceed)
    {
        if (viewer->display(convertedImage) == FrameworkReturnCode::_STOP)
        {
            proceed = false;
            std::cout << "end of ImageConvertor test" << std::endl;
        }
    }

    return 0;
}

int printHelp(){
        printf(" usage :\n");
        printf(" exe ImageFilePath \n");
        return 1;
}


int main(int argc, char *argv[]){
    if(argc==2)
        return run(argc,argv);
    else
        return(printHelp());
}
