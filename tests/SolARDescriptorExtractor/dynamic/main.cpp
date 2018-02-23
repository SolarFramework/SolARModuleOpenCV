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
#include "SolARModuleManagerNonFreeOpencv.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int run(int argc,char** argv)
{
    // instantiate module manager
    MODULES::OPENCV::SolARModuleManagerOpencv opencvModule(argv[2]);
    if (!opencvModule.isLoaded()) // xpcf library load has failed
    {
        LOG_ERROR("XPCF library load has failed")
        return -1;
    }

    MODULES::NONFREEOPENCV::SolARModuleManagerOpencvNonFree opencvNonFreeModule(argv[2]);
    if (!opencvNonFreeModule.isLoaded()) // xpcf library load has failed
    {
        LOG_ERROR("XPCF library load has failed")
        return -1;
    }    

 // declarations and creation of components
    SRef<image::IImageLoader> imageLoader = opencvModule.createComponent<image::IImageLoader>(MODULES::OPENCV::UUID::IMAGE_LOADER);
    SRef<display::IImageViewer> viewer = opencvModule.createComponent<display::IImageViewer>(MODULES::OPENCV::UUID::IMAGE_VIEWER);
    SRef<display::I2DOverlay> overlay = opencvModule.createComponent<display::I2DOverlay>(MODULES::OPENCV::UUID::OVERLAY2D);
    SRef<features::IKeypointDetector> keypointsDetector = opencvNonFreeModule.createComponent<features::IKeypointDetector>(MODULES::NONFREEOPENCV::UUID::KEYPOINT_DETECTOR_NONFREEOPENCV);
    SRef<features::IDescriptorsExtractor> extractorSIFT = opencvNonFreeModule.createComponent<features::IDescriptorsExtractor>(MODULES::NONFREEOPENCV::UUID::DESCRIPTORS_EXTRACTOR_SIFT);

    

    if (!imageLoader || !keypointsDetector || !extractorSIFT || !viewer || !overlay)
    {
        LOG_ERROR("One or more component creations have failed");
        return -1;
    }

    SRef<Image>                                        testImage;
    std::vector< sptrnms::shared_ptr<Keypoint>>        keypoints;
    SRef<DescriptorBuffer>                             descriptors;

    // The escape key to exit the sample
    char escape_key = 27;


    // components initialisation
        // nothing to do

 // Start
    if (imageLoader->loadImage(argv[1], testImage) != FrameworkReturnCode::_SUCCESS)
    {
       LOG_ERROR("Cannot load image with path {}", argv[1]);
       return -1;
    }

    keypointsDetector->detect(testImage, keypoints);
    extractorSIFT->extract(testImage, keypoints, descriptors);

    overlay->drawCircles(keypoints, 3, 1, testImage);

    bool proceed = true;
    while (proceed)
    {
        if (viewer->display("show keypoints", testImage, &escape_key) == FrameworkReturnCode::_STOP)
        {
            proceed = false;
            std::cout << "end of DescriptorsExtractorOpencv test" << std::endl;
        }
    }
    return 0;
}

int printHelp(){
        printf(" usage :\n");
        printf(" exe ImageFilePath configFilePath\n");
        printf(" Escape key to exit");
        return 1;
}

int main(int argc, char **argv){
    if(argc == 3){
        run(argc,argv);
         return 1;
    }
    else
        return(printHelp());
}


