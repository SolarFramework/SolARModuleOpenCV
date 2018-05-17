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

#include "IComponentManager.h"

#include "SolARModuleOpencv_traits.h"
#include "api/image/IImageLoader.h"
#include "api/features/IKeypointDetector.h"
#include "api/display/IImageViewer.h"
#include "api/display/ISideBySideOverlay.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IDescriptorsExtractor.h"

#include <iostream>
#include <string>
#include <vector>

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int run(int argc,char** argv)
{

    // load library
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();
    xpcfComponentManager->load("$BCOMDEVROOT/.xpcf",true);
    // instantiate module managers
    if (!xpcfComponentManager->isLoaded()) // xpcf library load has failed
    {
        LOG_ERROR("XPCF library load has failed")
        return -1;
    }


 // declarations and creation of components

    auto imageLoader1 = xpcfComponentManager->create<SolAR::MODULES::OPENCV::SolARImageLoaderOpencv>()->bindTo<image::IImageLoader>();
    auto imageLoader2 = xpcfComponentManager->create<SolAR::MODULES::OPENCV::SolARImageLoaderOpencv>()->bindTo<image::IImageLoader>();
    auto keypointsDetector = xpcfComponentManager->create<SolAR::MODULES::OPENCV::SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
    auto viewer = xpcfComponentManager->create<SolAR::MODULES::OPENCV::SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    auto overlay = xpcfComponentManager->create<SolAR::MODULES::OPENCV::SolARSideBySideOverlayOpencv>()->bindTo<display::ISideBySideOverlay>();
    auto matcher = xpcfComponentManager->create<SolAR::MODULES::OPENCV::SolARDescriptorMatcherHammingBruteForceOpencv>()->bindTo<features::IDescriptorMatcher>();
    auto extractorAKAZE = xpcfComponentManager->create<SolAR::MODULES::OPENCV::SolARDescriptorsExtractorAKAZEOpencv>()->bindTo<features::IDescriptorsExtractor>();

    if (!imageLoader1 || !imageLoader2 || !keypointsDetector || !extractorAKAZE || !matcher || !viewer || !overlay)
    {
        LOG_ERROR("One or more component creations have failed");
        return -1;
    }

    SRef<Image>                                        image1;
    SRef<Image>                                        image2;
    std::vector< sptrnms::shared_ptr<Keypoint>>        keypoints1;
    std::vector< sptrnms::shared_ptr<Keypoint>>        keypoints2;
    SRef<DescriptorBuffer>                             descriptors1;
    SRef<DescriptorBuffer>                             descriptors2;
    std::vector<DescriptorMatch>                       matches;
    std::vector<SRef<Point2Df>>                        matchedKeypoints1;
    std::vector<SRef<Point2Df>>                        matchedKeypoints2;
    SRef<Image>                                        viewerImage;

    // The escape key to exit the sample
    char escape_key = 27;

 // components initialisation
    // nothing to do

 // Start
    // Load the first image
    if (imageLoader1->loadImage(argv[1], image1) != FrameworkReturnCode::_SUCCESS)
    {
       LOG_ERROR("Cannot load image with path {}", argv[1]);
       return -1;
    }

    // Load the second image
    if (imageLoader2->loadImage(argv[2], image2) != FrameworkReturnCode::_SUCCESS)
    {
       LOG_ERROR("Cannot load image with path {}", argv[2]);
       return -1;
    }

    // Detect the keypoints of the first image
    keypointsDetector->detect(image1, keypoints1);

    // Detect the keypoints of the second image
    keypointsDetector->detect(image2, keypoints2);

    // Compute the AKAZE descriptor for each keypoint extracted from the first image
    extractorAKAZE->extract(image1, keypoints1, descriptors1);

    // Compute the AKAZE descriptor for each keypoint extracted from the second image
    extractorAKAZE->extract(image2, keypoints2, descriptors2);

    // Compute the matches between the keypoints of the first image and the keypoints of the second image
    matcher->match(descriptors1, descriptors2, matches);

    // Reorder the keypoints that match in to vector of 2D point (nth point of first vector matching with nth point of the second vector)
    // For OpenCV module testing. A keypoint reindexer component is available in the module "Tools"
    matchedKeypoints1.clear();
    matchedKeypoints2.clear();

    for( int i = 0; i < matches.size(); i++ )
    {
        matchedKeypoints1.push_back(xpcf::utils::make_shared<Point2Df>(keypoints1[ matches[i].getIndexInDescriptorA()]->getX(),keypoints1[ matches[i].getIndexInDescriptorA()]->getY()));
        matchedKeypoints2.push_back(xpcf::utils::make_shared<Point2Df>(keypoints2[ matches[i].getIndexInDescriptorB()]->getX(),keypoints2[ matches[i].getIndexInDescriptorB()]->getY()));
    }

    // Draw the matches in a dedicated image
    overlay->drawMatchesLines(image1, image2, viewerImage, matchedKeypoints1, matchedKeypoints2);

    bool proceed = true;
    while (proceed)
    {
        // Display the image with matches in a viewer. If escape key is pressed, exit the loop.
        if (viewer->display("show matches", viewerImage, &escape_key) == FrameworkReturnCode::_STOP)
        {
            proceed = false;
            LOG_INFO("End of DescriptorMatcherOpenCVStaticTest");
        }
    }

    return 0;
}


int printHelp(){
        printf(" usage :\n");
        printf(" exe firstImagePath secondImagePath\n");
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





