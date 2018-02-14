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
#include "SolARDescriptorsExtractorSURF128Opencv.h"

//#include "SolARDescriptorMatcherKNNOpencv.h"
#include "SolARDescriptorMatcherRadiusOpencv.h"
#include "SolARImageViewerOpencv.h"
#include "SolARSideBySideOverlayOpencv.h"
#include "SolARBasicMatchesFilterOpencv.h"
#include "SolARGeometricMatchesFilterOpencv.h"
#include "SolARFundamentalMatrixEstimationOpencv.h"
#include "SolARSVDFundamentalMatrixDecomposerOpencv.h"
#include "SolARSVDTriangulationOpencv.h"


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


void fillK(CamCalibration&cam){
    cam(0,0) = 2759.48;
    cam(0,1) = 0.0;
    cam(0,2) = 1520.69;

    cam(1,0) = 0.0;
    cam(1,1) = 2764.16;
    cam(1,2) = 1006.81;

    cam(2,0) = 0.0;
    cam(2,1) = 0.0;
    cam(2,2) = 1.0;
}
void fillDistorsion(CamDistortion&dist){
    for(int i = 0; i < 5; ++i){
        dist(i,0) = 0;
    }
    dist(4,0) = 0.0;
}
void fillPoseCanonique(SRef<Pose>&pcano){

    pcano = sptrnms::make_shared<Pose>();

    pcano->m_poseTransform(0,0) = 1.0;
    pcano->m_poseTransform(0,1) = 0.0;
    pcano->m_poseTransform(0,2) = 0.0;
    pcano->m_poseTransform(0,3) = 0.0;

    pcano->m_poseTransform(1,0) = 0.0;
    pcano->m_poseTransform(1,1) = 1.0;
    pcano->m_poseTransform(1,2) = 0.0;
    pcano->m_poseTransform(1,3) = 0.0;

    pcano->m_poseTransform(2,0) = 0.0;
    pcano->m_poseTransform(2,1) = 0.0;
    pcano->m_poseTransform(2,2) = 1.0;
    pcano->m_poseTransform(2,3) = 0.0;

    pcano->m_poseTransform(3,0) = 0.0;
    pcano->m_poseTransform(3,1) = 0.0;
    pcano->m_poseTransform(3,2) = 0.0;
    pcano->m_poseTransform(3,3) = 1.0;
}





int run(std::string& firstImagePath, std::string& secondImagePath, std::string& cameraParameters)
{

 // declarations
    xpcf::utils::uuids::string_generator                 gen;

	SRef<input::devices::ICamera>                        camera;

    SRef<image::IImageLoader>                            imageLoader;
    SRef<features::IKeypointDetector>                    keypointsDetector;
    SRef<features::IDescriptorsExtractor>                extractorSURF;

    SRef<features::IDescriptorMatcher>                   matcher;
    SRef<display::IImageViewer>                          viewer;

    SRef<solver::pose::IFundamentalMatrixEstimation>    fundamentalFinder;
    SRef<solver::pose::IFundamentalMatrixDecomposer>    fundamentalDecomposer;
    SRef<solver::map::ITriangulator>                    mapper;

    SRef<display::ISideBySideOverlay>                   overlay;

    SRef<Image>                                         image1;
    SRef<Image>                                         image2;

    std::vector< SRef<Keypoint>>                        keypoints1;
    std::vector< SRef<Keypoint>>                        keypoints2;


    SRef<DescriptorBuffer>                              descriptors1;
    SRef<DescriptorBuffer>                              descriptors2;
    std::vector<DescriptorMatch>                        matches;

    std::vector<SRef<Point2Df>>                         matchedKeypoints1;
    std::vector<SRef<Point2Df>>                         matchedKeypoints2;

    std::vector<SRef<Point2Df>>                         gmatchedKeypoints1;
    std::vector<SRef<Point2Df>>                         gmatchedKeypoints2;

    std::vector<SRef<Point2Df>>                         ggmatchedKeypoints1;
    std::vector<SRef<Point2Df>>                         ggmatchedKeypoints2;
    std::vector<SRef<Point3Df>>                         gcloud;



    SRef<Image>                                         viewerImage1;
    SRef<Image>                                         viewerImage2;
    SRef<Image>                                         viewerImage3;

    SRef<features::IMatchesFilter>                      matchesFilterBasic;
    SRef<features::IMatchesFilter>                      matchesFilterGeometric;
    std::vector<DescriptorMatch>                        gmatches;
    std::vector<DescriptorMatch>                        ggmatches;

    CamCalibration                                      K;
    CamDistortion                                       dist;
    Transform2Df                                        F;
    std::vector<SRef<Pose>>                             poses;
    // The escape key to exit the sample
    char escape_key = 27;

 // component creation

	xpcf::ComponentFactory::createComponent<SolARCameraOpencv>(gen(input::devices::ICamera::UUID), camera);
	xpcf::ComponentFactory::createComponent<SolARImageLoaderOpencv>(gen(image::IImageLoader::UUID ), imageLoader);
    xpcf::ComponentFactory::createComponent<SolARKeypointDetectorOpencv>(gen(features::IKeypointDetector::UUID ), keypointsDetector);
    xpcf::ComponentFactory::createComponent<SolARDescriptorsExtractorSURF128Opencv>(gen(features::IDescriptorsExtractor::UUID ), extractorSURF);
    xpcf::ComponentFactory::createComponent<SolARDescriptorMatcherRadiusOpencv>(gen(features::IDescriptorMatcher::UUID ), matcher);
    xpcf::ComponentFactory::createComponent<SolARSideBySideOverlayOpencv>(gen(display::ISideBySideOverlay::UUID ), overlay);
    xpcf::ComponentFactory::createComponent<SolARImageViewerOpencv>(gen(display::IImageViewer::UUID ), viewer);
    xpcf::ComponentFactory::createComponent<SolARBasicMatchesFilterOpencv>(gen(features::IMatchesFilter::UUID ),
                                                                            matchesFilterBasic);
    xpcf::ComponentFactory::createComponent<SolARGeometricMatchesFilterOpencv>(gen(features::IMatchesFilter::UUID ),
                                                                            matchesFilterGeometric);

    xpcf::ComponentFactory::createComponent<SolARFundamentalMatrixEstimationOpencv>(gen(solver::pose::IFundamentalMatrixEstimation::UUID ),
                                                                           fundamentalFinder);

    xpcf::ComponentFactory::createComponent<SolARSVDFundamentalMatrixDecomposerOpencv>(gen(solver::pose::IFundamentalMatrixDecomposer::UUID ),
                                                                                        fundamentalDecomposer);

    xpcf::ComponentFactory::createComponent<SolARSVDTriangulationOpencv>(gen(solver::map::ITriangulator::UUID ),
                                                                                        mapper);


	// load camera parameters from yml input file
	camera->loadCameraParameters(cameraParameters);

     keypointsDetector->setType(KeypointDetectorType::SURF);
  //   keypointsDetector->setType(KeypointDetectorType::SURF);

   // Load the first image
   if (imageLoader->loadImage(firstImagePath,
                               image1) != FrameworkReturnCode::_SUCCESS)
   {
      LOG_ERROR("Cannot load image with path {}", firstImagePath);
      return -1;
   }

   // Load the second image
   if (imageLoader->loadImage(secondImagePath, image2) != FrameworkReturnCode::_SUCCESS)
   {
      LOG_ERROR("Cannot load image with path {}", std::string());
      return -1;
   }

   // Detect the keypoints of the first image
   keypointsDetector->detect(image1, keypoints1);

   // Detect the keypoints of the second image
   keypointsDetector->detect(image2, keypoints2);

   // Compute the SURF descriptor for each keypoint extracted from the first image
   extractorSURF->extract(image1, keypoints1, descriptors1);

   // Compute the SURF descriptor for each keypoint extracted from the second image
   extractorSURF->extract(image2, keypoints2, descriptors2);

   // Compute the matches between the keypoints of the first image and the keypoints of the second image
   matcher->match(descriptors1, descriptors2, matches);

  int vizPoints0 = int(matches.size()/3);
   std::cout<<"->original matches: "<<matches.size()<<std::endl;

   matchedKeypoints1.clear();
   matchedKeypoints2.clear();


   for( int i = 0; i < matches.size(); i++ ){
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

   //trying to estimate fundamental matrix;

    fundamentalFinder->findFundamental(ggmatchedKeypoints1, ggmatchedKeypoints2,F);
    std::cout<<"->F: "<<std::endl;
    for(int ii = 0; ii < 3; ++ii){
        for(int jj = 0; jj < 3; ++jj){
            std::cout<<F(ii,jj)<<" ";
        }
        std::cout<<std::endl;
    }

    fillK(K);
	fillDistorsion(dist);

    std::cout<<"->K: "<<std::endl;
    for(int ii = 0; ii < 3; ++ii){
        for(int jj = 0; jj < 3; ++jj){
            std::cout<<K(ii,jj)<<" ";
        }
        std::cout<<std::endl;
    }
    fundamentalDecomposer->decompose(F,K,poses);

    std::cout<<" Poses size: "<<poses.size()<<std::endl;

    for(int k = 0; k <poses.size(); ++k){
        std::cout<<"--pose: "<<k<<std::endl;
        for(int ii = 0; ii < 4; ++ii){
            for(int jj = 0; jj < 4; ++jj){
                std::cout<<poses[k]->m_poseTransform(ii,jj)<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl<<std::endl;
    }

    std::cout<<"-Triangulate: "<<std::endl;


   SRef<Pose>pose_canonique ; //= sptrnms::make_shared<Pose>();
   fillPoseCanonique(pose_canonique);

    for( int k = 0; k < poses.size(); ++k){
     std::cout<<"    - with pose: "<<k<<std::endl;

     for(int ii = 0; ii < 4; ++ii){
         for(int jj = 0; jj < 4; ++jj){
            std::cout<<poses[k]->m_poseTransform(ii,jj)<<" ";
         }
         std::cout<<std::endl;
      }
      std::cout<<std::endl<<std::endl;
      std::cout<<"    - and canonique"<<std::endl;

     for(int ii = 0; ii < 4; ++ii){
         for(int jj = 0; jj < 4; ++jj){
             std::cout<<pose_canonique->m_poseTransform(ii,jj)<<" ";
          }
          std::cout<<std::endl;
      }
     std::cout<<std::endl<<std::endl;
     mapper->triangulate(ggmatchedKeypoints1,ggmatchedKeypoints2,pose_canonique,poses[k],K,dist,gcloud);

     std::cout<<"--saving cloud: "<<std::endl;
    std::ofstream log_cloud("solar_cloud" +std::to_string(k)+ ".txt");
    log_cloud<<gcloud.size()<<std::endl;
     for(int kk = 0; kk < gcloud.size(); ++kk){
            log_cloud<<gcloud[kk]->getX()<<" "<<gcloud[kk]->getY()<<" "<<gcloud[kk]->getZ()<<std::endl;
     }
     log_cloud.close();
     gcloud.clear();
    }

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
        printf(" exe firstImagePath secondImagePath camaraParameters\n");
        return 1;
}

int main(int argc, char **argv){

	if (argc == 4) {
		std::string firstImagePath = std::string(argv[1]);
		std::string secondImagePath = std::string(argv[2]);
		std::string camaraParameters = std::string(argv[3]);

		run(firstImagePath, secondImagePath, camaraParameters);

		return 1;
	}
	else
		return(printHelp());

}



