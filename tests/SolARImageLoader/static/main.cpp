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

// ADD COMPONENTS HEADERS HERE, e.g #include "SolarComponent.h"

#include "SolARImageLoaderOpencv.h"
#include "SolARImageViewerOpencv.h"


using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;

namespace xpcf  = org::bcom::xpcf;

int run(int argc, char *argv[])
{
    // components declarations
    SRef<image::IImageLoader> imageLoader;
    SRef<display::IImageViewer> viewer;

    SRef<xpcf::IConfigurable> rIConfigurable;

    // components creation
    xpcf::ComponentFactory::createComponent<SolARImageLoaderOpencv>(xpcf::toUUID<image::IImageLoader>(), imageLoader);
    xpcf::ComponentFactory::createComponent<SolARImageViewerOpencv>(xpcf::toUUID<display::IImageViewer>(), viewer);

    SRef<Image> image;

    // The escape key to exit the sample
    char escape_key = 27;

    // USE your components here, e.g SolarComponentInstance->testMethod();
    if (imageLoader->loadImage(argv[1], image) != FrameworkReturnCode::_SUCCESS)
    {
       LOG_ERROR("Cannot load image with path {}", argv[1]);
       return -1;
    }
    
    bool proceed = true;
    while (proceed)
    {
        if (viewer->display("show image", image, &escape_key) == FrameworkReturnCode::_STOP)
        {
            proceed = false;
            std::cout << "end of SolARImageopenCV test" << std::endl;
        }
    }


    //leave
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
