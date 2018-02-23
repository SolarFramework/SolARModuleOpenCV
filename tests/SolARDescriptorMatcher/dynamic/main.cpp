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

#include "SolARModuleManagerOpencv.h"
#include "SolARModuleManagerNonFreeOpencv.h"

#include <iostream>

#include <string>
#include <vector>

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int run(int argc,char** argv)
{
    // instantiate module manager
    MODULES::OPENCV::SolARModuleManagerOpencv opencvModule(argv[3]);
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
    SRef<image::IImageLoader> imageLoader1 = opencvModule.createComponent<image::IImageLoader>(MODULES::OPENCV::UUID::IMAGE_LOADER);
    SRef<image::IImageLoader> imageLoader2 = opencvModule.createComponent<image::IImageLoader>(MODULES::OPENCV::UUID::IMAGE_LOADER);
    SRef<features::IDescriptorMatcher> matcher = opencvModule.createComponent<features::IDescriptorMatcher>(MODULES::OPENCV::UUID::DESCRIPTOR_MATCHER_KNN);
    SRef<display::IImageViewer> viewer = opencvModule.createComponent<display::IImageViewer>(MODULES::OPENCV::UUID::IMAGE_VIEWER);
    SRef<display::ISideBySideOverlay> overlay = opencvModule.createComponent<display::ISideBySideOverlay>(MODULES::OPENCV::UUID::OVERLAYSBS);
    SRef<features::IKeypointDetector> keypointsDetector = opencvNonFreeModule.createComponent<features::IKeypointDetector>(MODULES::NONFREEOPENCV::UUID::KEYPOINT_DETECTOR_NONFREEOPENCV);
    SRef<features::IDescriptorsExtractor> extractorSIFT = opencvNonFreeModule.createComponent<features::IDescriptorsExtractor>(MODULES::NONFREEOPENCV::UUID::DESCRIPTORS_EXTRACTOR_SIFT);    

    if (!imageLoader1 || !imageLoader2 || !keypointsDetector || !extractorSIFT || !matcher || !viewer || !overlay)
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

    // Compute the SIFT descriptor for each keypoint extracted from the first image
    extractorSIFT->extract(image1, keypoints1, descriptors1);

    // Compute the SIFT descriptor for each keypoint extracted from the second image
    extractorSIFT->extract(image2, keypoints2, descriptors2);

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
        printf(" exe firstImagePath secondImagePath configFilePath\n");
        return 1;
}


int main(int argc, char **argv){
    if(argc == 4){
        run(argc,argv);
         return 1;
    }
    else
        return(printHelp());
}





