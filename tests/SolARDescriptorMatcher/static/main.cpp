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
    xpcf::utils::uuids::string_generator    gen;
    SRef<image::IImageLoader>               imageLoader1;
    SRef<image::IImageLoader>               imageLoader2;
    SRef<features::IKeypointDetector>       keypointsDetector;
    SRef<features::IDescriptorsExtractor>   extractorAKAZE;
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
    SRef<Image>                             viewerImage;

    // The escape key to exit the sample
    char escape_key = 27;

 // component creation
    xpcf::ComponentFactory::createComponent<SolARImageLoaderOpencv>(gen(image::IImageLoader::UUID ), imageLoader1);
    xpcf::ComponentFactory::createComponent<SolARImageLoaderOpencv>(gen(image::IImageLoader::UUID ), imageLoader2);
    xpcf::ComponentFactory::createComponent<SolARKeypointDetectorOpencv>(gen(features::IKeypointDetector::UUID ), keypointsDetector);
    xpcf::ComponentFactory::createComponent<SolARDescriptorsExtractorAKAZEOpencv>(gen(features::IDescriptorsExtractor::UUID ), extractorAKAZE);
    xpcf::ComponentFactory::createComponent<SolARDescriptorMatcherHammingBruteForceOpencv>(gen(features::IDescriptorMatcher::UUID ), matcher);
    xpcf::ComponentFactory::createComponent<SolARSideBySideOverlayOpencv>(gen(display::ISideBySideOverlay::UUID ), overlay);
    xpcf::ComponentFactory::createComponent<SolARImageViewerOpencv>(gen(display::IImageViewer::UUID ), viewer);

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

    keypointsDetector->setType(KeypointDetectorType::AKAZE);
   // Detect the keypoints of the first image
 
   keypointsDetector->detect(image1, keypoints1);

   // Detect the keypoints of the second image
    keypointsDetector->detect(image2, keypoints2);

    int size_k1 = keypoints1.size();
    int size_k2 = keypoints2.size();
    
    std::cout<<"keypoints size:"<<size_k1<<" "<<size_k2 <<std::endl;
    
   // Compute the SIFT descriptor for each keypoint extracted from the first image
   extractorAKAZE->extract(image1, keypoints1, descriptors1);

   // Compute the SIFT descriptor for each keypoint extracted from the second image
   extractorAKAZE->extract(image2, keypoints2, descriptors2);

#ifdef cvhack
    std::vector<cv::KeyPoint> kpts1, kpts2;

    for(unsigned int k =0; k < keypoints1.size(); ++k)
    {
        kpts1.push_back(
                    //instantiate keypoint
                     cv::KeyPoint(keypoints1[k]->getX(),
                                  keypoints1[k]->getY(),
                                  keypoints1[k]->getSize(),
                                  keypoints1[k]->getAngle(),
                                  keypoints1[k]->getResponse(),
                                  keypoints1[k]->getOctave(),
                                  keypoints1[k]->getClassId())
                    );
    }

    for(unsigned int k =0; k < keypoints2.size(); ++k)
    {
        kpts2.push_back(
                    //instantiate keypoint
                     cv::KeyPoint(keypoints2[k]->getX(),
                                  keypoints2[k]->getY(),
                                  keypoints2[k]->getSize(),
                                  keypoints2[k]->getAngle(),
                                  keypoints2[k]->getResponse(),
                                  keypoints2[k]->getOctave(),
                                  keypoints2[k]->getClassId())
                    );
    }


   
    uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptors1->getDescriptorDataType());
    cv::Mat desc1(descriptors1->getNbDescriptors(), descriptors1->getNbElements(), type_conversion);
    desc1.data=(uchar*)descriptors1->data();
   
    cv::Mat desc2(descriptors2->getNbDescriptors(), descriptors2->getNbElements(), type_conversion);
    desc2.data=(uchar*)descriptors2->data();

    BFMatcher matcher2(NORM_HAMMING);
    vector< vector<DMatch> > nn_matches;
    matcher2.knnMatch(desc1, desc2, nn_matches, 2);
    vector<KeyPoint> matched1, matched2, inliers1, inliers2;
    vector<DMatch> good_matches;
////

    for(unsigned i = 0; i < nn_matches.size(); i++) {
        if(nn_matches[i][0].distance < nn_match_ratio * nn_matches[i][1].distance) {
            matched1.push_back(kpts1[nn_matches[i][0].queryIdx]);
            matched2.push_back(kpts2[nn_matches[i][0].trainIdx]);
        }
    }
    Mat homography;
    Mat inlier_mask;
    vector<cv::DMatch> inlier_matches;
    /*
    if(matched1.size() >= 4) {
        homography = findHomography(Points(matched1), Points(matched2),
                                    cv::RANSAC, ransac_thresh, inlier_mask);
        homography.convertTo(homography,CV_32F);
    }

    for(unsigned i = 0; i < matched1.size(); i++) {
        if(inlier_mask.at<uchar>(i)) {
            int new_i = static_cast<int>(inliers1.size());
            inliers1.push_back(matched1[i]);
            inliers2.push_back(matched2[i]);
            inlier_matches.push_back(DMatch(new_i, new_i, 0));
        }
    }
    */
    Mat res;

    cv::Mat cv_image1,cv_image2;
    SolAROpenCVHelper::mapToOpenCV(image1,cv_image1);
    SolAROpenCVHelper::mapToOpenCV(image2,cv_image2);

    // all matches
    cv::drawMatches(cv_image1, kpts1, cv_image2, kpts2, nn_matches,res,Scalar(255, 0, 0), Scalar(255, 0, 0));
    cv::imshow("matches", res);

    // only inliers
    cv::drawMatches(cv_image1, inliers1, cv_image2, inliers2,
                inlier_matches, res,
                Scalar(255, 0, 0), Scalar(255, 0, 0));
    cv::imshow("inliers", res);
    while(1)
        if(waitKey(1)==27) break; //quit on ESC button
////

    imwrite("res.png", res);
    double inlier_ratio = inliers1.size() * 1.0 / matched1.size();
    cout << "A-KAZE Matching Results" << endl;
    cout << "*******************************" << endl;
    cout << "# Keypoints 1:                        \t" << keypoints1.size() << endl;
    cout << "# Keypoints 2:                        \t" << keypoints2.size() << endl;
    cout << "# Matches:                            \t" << matched1.size() << endl;
    cout << "# Inliers:                            \t" << inliers1.size() << endl;
    cout << "# Inliers Ratio:                      \t" << inlier_ratio << endl;
    cout << endl;

#endif


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



