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
#include "SolARDescriptorsExtractorSIFTOpencv.h"
//#include "SolARDescriptorMatcherKNNOpencv.h"
#include "SolARDescriptorMatcherRadiusOpencv.h"
#include "SolARImageViewerOpencv.h"
#include "SolARSideBySideOverlayOpencv.h"
#include "SolARBasicMatchesFilterOpencv.h"
#include "SolARGeometricMatchesFilterOpencv.h"

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#include "opencv2/opencv_modules.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;

namespace xpcf  = org::bcom::xpcf;


int run(int argc, char **argv)
{

 // declarations
    xpcf::utils::uuids::string_generator    gen;
    SRef<image::IImageLoader>               imageLoader;
    SRef<features::IKeypointDetector>       keypointsDetector;
    SRef<features::IDescriptorsExtractor>   extractorSIFT;
    SRef<features::IDescriptorMatcher>      matcher;
    SRef<display::IImageViewer>             viewer;

    SRef<display::ISideBySideOverlay>       overlay;

    SRef<Image>                             image1;
    SRef<Image>                             image2;

    std::vector< SRef<Keypoint>>            keypoints1;
    std::vector< SRef<Keypoint>>            keypoints2;


    SRef<DescriptorBuffer>                  descriptors1;
    SRef<DescriptorBuffer>                  descriptors2;
    std::vector<DescriptorMatch>            matches;

    std::vector<SRef<Point2Df>>             matchedKeypoints1;
    std::vector<SRef<Point2Df>>             matchedKeypoints2;

    std::vector<SRef<Point2Df>>             gmatchedKeypoints1;
    std::vector<SRef<Point2Df>>             gmatchedKeypoints2;

    std::vector<SRef<Point2Df>>             ggmatchedKeypoints1;
    std::vector<SRef<Point2Df>>             ggmatchedKeypoints2;


    SRef<Image>                             viewerImage1;
    SRef<Image>                             viewerImage2;
    SRef<Image>                             viewerImage3;

    SRef<features::IMatchesFilter>          matchesFilterBasic;
    SRef<features::IMatchesFilter>          matchesFilterGeometric;
    std::vector<DescriptorMatch>            gmatches;
    std::vector<DescriptorMatch>            ggmatches;

    // The escape key to exit the sample
    char escape_key = 27;

 // component creation
    xpcf::ComponentFactory::createComponent<SolARImageLoaderOpencv>(gen(image::IImageLoader::UUID ), imageLoader);
    xpcf::ComponentFactory::createComponent<SolARKeypointDetectorOpencv>(gen(features::IKeypointDetector::UUID ), keypointsDetector);
    xpcf::ComponentFactory::createComponent<SolARDescriptorsExtractorSIFTOpencv>(gen(features::IDescriptorsExtractor::UUID ), extractorSIFT);
    xpcf::ComponentFactory::createComponent<SolARDescriptorMatcherRadiusOpencv>(gen(features::IDescriptorMatcher::UUID ), matcher);
    xpcf::ComponentFactory::createComponent<SolARSideBySideOverlayOpencv>(gen(display::ISideBySideOverlay::UUID ), overlay);
    xpcf::ComponentFactory::createComponent<SolARImageViewerOpencv>(gen(display::IImageViewer::UUID ), viewer);
    xpcf::ComponentFactory::createComponent<SolARBasicMatchesFilterOpencv>(gen(features::IMatchesFilter::UUID ),
                                                                            matchesFilterBasic);
     xpcf::ComponentFactory::createComponent<SolARGeometricMatchesFilterOpencv>(gen(features::IMatchesFilter::UUID ),
                                                                            matchesFilterGeometric);

     keypointsDetector->setType(KeypointDetectorType::SIFT);
   // Load the first image
   if (imageLoader->loadImage(argv[1],
                               image1) != FrameworkReturnCode::_SUCCESS)
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

  int vizPoints0 = int(matches.size()/3);
   std::cout<<"->original matches: "<<matches.size()<<std::endl;

   matchedKeypoints1.clear();
   matchedKeypoints2.clear();


   for( int i = 0; i < matches.size(); i++ )
   {
       matchedKeypoints1.push_back(xpcf::utils::make_shared<Point2Df>(keypoints1[ matches[i].getIndexInDescriptorA()]->getX(),keypoints1[ matches[i].getIndexInDescriptorA()]->getY()));
       matchedKeypoints2.push_back(xpcf::utils::make_shared<Point2Df>(keypoints2[ matches[i].getIndexInDescriptorB()]->getX(),keypoints2[ matches[i].getIndexInDescriptorB()]->getY()));
   }

   // Draw the matches in a dedicated image
   overlay->drawMatchesLines(image1, image2, viewerImage1, matchedKeypoints1, matchedKeypoints2, vizPoints0);
   matchesFilterBasic->filter(matches,gmatches,keypoints1, keypoints2);
   std::cout<<"->filtred matches with redanduncy: "<<gmatches.size()<<std::endl;

    gmatchedKeypoints1.clear();
    gmatchedKeypoints2.clear();

    for( int i = 0; i < gmatches.size(); i++ ){
       gmatchedKeypoints1.push_back(xpcf::utils::make_shared<Point2Df>(keypoints1[gmatches[i].getIndexInDescriptorA()]->getX(),keypoints1[ gmatches[i].getIndexInDescriptorA()]->getY()));
       gmatchedKeypoints2.push_back(xpcf::utils::make_shared<Point2Df>(keypoints2[gmatches[i].getIndexInDescriptorB()]->getX(),keypoints2[ gmatches[i].getIndexInDescriptorB()]->getY()));
    }
    int vizPoints1 = int(gmatches.size()/2.5);
     overlay->drawMatchesLines(image1, image2, viewerImage2, gmatchedKeypoints1, gmatchedKeypoints2,vizPoints1);

    matchesFilterGeometric->filter(gmatches,ggmatches,keypoints1, keypoints2);
    std::cout<<"->filtred matches with epipolar constraint: "<<ggmatches.size()<<std::endl;

    ggmatchedKeypoints1.clear();
    ggmatchedKeypoints2.clear();

    for( int i = 0; i < ggmatches.size(); i++ ){
       ggmatchedKeypoints1.push_back(xpcf::utils::make_shared<Point2Df>(keypoints1[ggmatches[i].getIndexInDescriptorA()]->getX(),keypoints1[ ggmatches[i].getIndexInDescriptorA()]->getY()));
       ggmatchedKeypoints2.push_back(xpcf::utils::make_shared<Point2Df>(keypoints2[ggmatches[i].getIndexInDescriptorB()]->getX(),keypoints2[ ggmatches[i].getIndexInDescriptorB()]->getY()));
    }
    int vizPoints2 = int(ggmatches.size());
    overlay->drawMatchesLines(image1, image2, viewerImage3, ggmatchedKeypoints1, ggmatchedKeypoints2,vizPoints2);

   bool process = true;
   while (process){
       viewer->display("original matches", viewerImage1,1280,480);
       viewer->display("filtred matches (redanduncy)", viewerImage2,1280,480);
       viewer->display("filtred matches (epipolar)", viewerImage3,1280,480);
       if(cv::waitKey(0) == 27){
           process = false;
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



