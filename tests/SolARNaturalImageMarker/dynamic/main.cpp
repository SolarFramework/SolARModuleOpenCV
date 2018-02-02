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

#include "SolARModuleManagerOpencv.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

int run(int argc,char* argv[])
{
    // instantiate module manager
    MODULES::OPENCV::SolARModuleManagerOpencv opencvModule(argv[2]);
    if (!opencvModule.isLoaded()) // xpcf library load has failed
    {
        LOG_ERROR("XPCF library load has failed")
        return -1;
    }

    // declarations and creation of components
       SRef<input::files::IMarker2DNaturalImage> naturalImageMarker = opencvModule.createComponent<input::files::IMarker2DNaturalImage>(MODULES::OPENCV::UUID::MARKER2D_NATURAL_IMAGE);
       SRef<display::IImageViewer> viewer = opencvModule.createComponent<display::IImageViewer>(MODULES::OPENCV::UUID::IMAGE_VIEWER);

       if (!naturalImageMarker || !viewer )
       {
           LOG_ERROR("One or more component creations have failed");
           return -1;
       }

       SRef<Image> markerImage;

       // The escape key to exit the sample
       char escape_key = 27;

       // components initialisation
           // nothing to do

       // Start
       naturalImageMarker->loadMarker(argv[1]);

       LOG_TRACE("Marker size width: {}", naturalImageMarker->getWidth());
       LOG_TRACE("Marker size heigth: {}", naturalImageMarker->getHeight());

       naturalImageMarker->getImage(markerImage);

       bool proceed = true;
       while (proceed)
       {
           if (viewer->display("Show Natural Image Marker", markerImage, &escape_key) == FrameworkReturnCode::_STOP)
           {
               proceed = false;
               std::cout << "end of DescriptorsExtractorOpencv test" << std::endl;
           }
       }
       return 0;
}

int printHelp(){
        printf(" usage :\n");
        printf(" exe naturalImagemarkerFilename configFilePath\n");
        return 1;
}


int main(int argc, char *argv[]){
    if(argc==3)
        return run(argc,argv);
    else
        return(printHelp());
}

