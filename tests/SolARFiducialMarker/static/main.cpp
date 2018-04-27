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

#include "SolARMarker2DSquaredBinaryOpencv.h"

namespace xpcf  = org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;

int run1(int argc,char* argv[])
{
    // declarations
    SRef<input::files::IMarker2DSquaredBinary> binaryMarker;

    // component creation
    xpcf::ComponentFactory::createComponent<SolARMarker2DSquaredBinaryOpencv>(xpcf::toUUID<input::files::IMarker2DSquaredBinary>(), binaryMarker);


    // components initialisation: here, load the marker informatiosn from a decsription file
    if (binaryMarker->loadMarker(std::string(argv[1]))!= SolAR::FrameworkReturnCode::_SUCCESS)
    {
            std::cout<<"Pattern maker "<<argv[1]<<" cannot be loaded";
            return 1;
    }

    const SquaredBinaryPatternMatrix pattern = binaryMarker->getPattern()->getPatternMatrix();

    // Display the world size of the marker in the console
    std::cout<<"Marker size width: "<<binaryMarker->getWidth()<<std::endl;
    std::cout<<"Marker size height: "<<binaryMarker->getHeight()<<std::endl;

    // Display the pattern information in the console, w=white=1 case, b=black=0 case.
    std::cout<<"Marker pattern: "<<std::endl;
    for (int i= 0; i < (int)pattern.cols(); i++)
    {
        std::cout<<"[";
        for (int j = 0; j < (int)pattern.rows(); j++)
        {
            if (pattern(i,j))
                std::cout<<"w ";
            else
                std::cout<<"b ";
        }
        std::cout<<"]"<<std::endl;;
    }

    return 0;

}

int printHelp(){
        printf(" usage :\n");
        printf(" exe SquaredBinaryMarkerFilename \n");
        return 1;
}


int main(int argc, char *argv[]){
    if(argc==2)
        return run1(argc,argv);
    else
        return(printHelp());
}



