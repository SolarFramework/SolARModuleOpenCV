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

// ADD COMPONENTS HEADERS HERE

#include "SolARImageLoaderOpencv.h"
#include "SolARCameraOpencv.h"
#include "SolARKeypointDetectorOpencv.h"
#include "SolARDescriptorMatcherKNNOpencv.h"
#include "SolARDescriptorMatcherHammingBruteForceOpencv.h"
#include "SolARImageViewerOpencv.h"
#include "SolARSideBySideOverlayOpencv.h"

#include "SolARDescriptorsExtractorAKAZEOpencv.h"

#include "SolAROpenCVHelper.h"

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;


namespace xpcf  = org::bcom::xpcf;

using namespace std;
using namespace cv;
const float inlier_threshold = 2.5f; // Distance threshold to identify inliers
const float nn_match_ratio = 0.8f;   // Nearest neighbor matching ratio
const double ransac_thresh = 2.5f; // RANSAC inlier threshold
const double akaze_thresh = 3e-4; // AKAZE detection threshold set to locate about 1000 keypoints


int run(int argc,char** argv)
{
 // declarations

    SRef<Image>                             image1;
    SRef<Image>                             image2;
    std::vector< SRef<Keypoint>>            keypoints1;
    std::vector< SRef<Keypoint>>            keypoints2;
    SRef<DescriptorBuffer>                  descriptors1;
    SRef<DescriptorBuffer>                  descriptors2;
    std::vector<DescriptorMatch>            matches;
    std::vector<SRef<Point2Df>>             matchedKeypoints1;
    std::vector<SRef<Point2Df>>             matchedKeypoints2;
    SRef<Image>                             viewerImage;

    // The escape key to exit the sample
    char escape_key = 27;

 // component creation
    auto imageLoader=xpcf::ComponentFactory::createInstance<SolARImageLoaderOpencv>()->bindTo<image::IImageLoader>();
    auto keypointsDetector =xpcf::ComponentFactory::createInstance<SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
    auto descriptorExtractor = xpcf::ComponentFactory::createInstance<SolARDescriptorsExtractorAKAZEOpencv>()->bindTo<features::IDescriptorsExtractor>();
    auto  matcher =xpcf::ComponentFactory::createInstance<SolARDescriptorMatcherHammingBruteForceOpencv>()->bindTo<features::IDescriptorMatcher>();
    auto  overlay =xpcf::ComponentFactory::createInstance<SolARSideBySideOverlayOpencv>()->bindTo<display::ISideBySideOverlay>();
    auto  viewer =xpcf::ComponentFactory::createInstance<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();

 // components initialisation
    // nothing to do

// Start
   // Load the first image
   if (imageLoader->loadImage(argv[1], image1) != FrameworkReturnCode::_SUCCESS)
   {
      LOG_ERROR("Cannot load image with path {}", argv[1]);
      return -1;
   }

   // Load the second image
   if (imageLoader->loadImage(argv[2], image2) != FrameworkReturnCode::_SUCCESS)
   {
      LOG_ERROR("Cannot load image with path {}", argv[2]);
      return -1;
   }

    keypointsDetector->setType(KeypointDetectorType::AKAZE);
   // Detect the keypoints of the first image

   keypointsDetector->detect(image1, keypoints1);

   // Detect the keypoints of the second image
    keypointsDetector->detect(image2, keypoints2);

    int size_k1 = keypoints1.size();
    int size_k2 = keypoints2.size();

   // Compute the AKAZE descriptor for each keypoint extracted from the first image
   descriptorExtractor->extract(image1, keypoints1, descriptors1);

   // Compute the AKAZE descriptor for each keypoint extracted from the second image
   descriptorExtractor->extract(image2, keypoints2, descriptors2);

    int size_d1 =  descriptors1->getNbDescriptors();
    int size_d2 = descriptors2->getNbDescriptors();

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
    if(argc == 3){
        run(argc,argv);
         return 1;
    }
    else
        return(printHelp());
}



