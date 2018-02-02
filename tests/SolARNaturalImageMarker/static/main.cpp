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

// ADD COMPONENTS HEADERS HERE, e.g #include "SolarComponent.h"

#include "SolARMarker2DNaturalImageOpencv.h"
#include "SolARImageViewerOpencv.h"

namespace xpcf  = org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;

int run1(int argc,char* argv[])
{
    // declarations

    bool verbose=true;
    char key=' ';

    boost::uuids::string_generator gen;

    SRef<input::files::IMarker2DNaturalImage> naturalImageMarker;
    SRef<display::IImageViewer> imageViewer;

    SRef<Image> markerImage;

    // component creation

    xpcf::ComponentFactory::createComponent<SolARMarker2DNaturalImageOpencv>(gen(input::files::IMarker2DNaturalImage::UUID ),naturalImageMarker);
    xpcf::ComponentFactory::createComponent<SolARImageViewerOpencv>(gen(display::IImageViewer::UUID ),imageViewer);

    // components initialisation
    if (naturalImageMarker->loadMarker(std::string(argv[0]))!= SolAR::FrameworkReturnCode::_SUCCESS)
    {
            std::cout<<"Image maker "<<argv[0]<<" cannot be loaded";
            return 1;
    }

    if(argc==3)
        naturalImageMarker->setSize(std::stof(argv[1]), std::stof(argv[2])); //

    std::cout<<"Marker size width: "<<naturalImageMarker->getWidth()<<"\n";
    std::cout<<"Marker size height: "<<naturalImageMarker->getHeight()<<"\n";

    // display marker image
    char escape_key = 27;
    if (naturalImageMarker->getImage(markerImage)==SolAR::FrameworkReturnCode::_SUCCESS)
    {
        std::cout<<"Image resolution width: "<< markerImage->getWidth() << "\n";
        std::cout<<"Image resolution height: "<< markerImage->getHeight() << "\n";

        while (imageViewer->display("Marker",markerImage, &escape_key) != FrameworkReturnCode::_STOP){};
    }
    return 0;

}

int printHelp(){
        printf(" usage :\n");
        printf(" exe naturalImagemarkerFilename \n");
        return 1;
}


int main(int argc, char *argv[]){
    if(argc==2)
        return run1(argc-1,&argv[1]);
    else
        return(printHelp());
}



