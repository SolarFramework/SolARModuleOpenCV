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

#include "SolARFundamentalMatrixEstimationOpencv.h"


using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;

namespace xpcf  = org::bcom::xpcf;

void load_2dpoints(std::string&path_file, int points_no, std::vector<SRef<Point2Df>>&pt2d){
    cv::namedWindow("toto debug",0);
    std::ifstream ox(path_file);
    float pt[2];
  //  Point2Df point_temp;
    std::string dummy;
    pt2d.resize(points_no);
    float v[2];
    for(int i = 0; i < points_no; ++i){
       ox>>dummy;
       v[0]  = std::stof(dummy);
       ox>>dummy;
       v[1]= std::stof(dummy);
       pt2d[i]  = sptrnms::make_shared<Point2Df>(v[0], v[1]);
    }
  ox.close();
}
int run( std::string& path_points1, std::string& path_points2,std::string& path_fileout)
{

 // declarations
    xpcf::utils::uuids::string_generator              gen;
    SRef<solver::pose::IFundamentalMatrixEstimation>  fundamentalFinder;
    SRef<display::IImageViewer>                       viewer;
    SRef<display::ISideBySideOverlay>                 overlay;
    SRef<Image>                                       image;


    Transform2Df F;



    std::vector<SRef<Point2Df>>                      points_view1;
    std::vector<SRef<Point2Df>>                      points_view2;




    // The escape key to exit the sample
    char escape_key = 27;

 // component creation
   xpcf::ComponentFactory::createComponent<SolARImageViewerOpencv>(gen(display::IImageViewer::UUID ), viewer);
   xpcf::ComponentFactory::createComponent<SolARFundamentalMatrixEstimationOpencv>(gen(solver::pose::IFundamentalMatrixEstimation::UUID ), fundamentalFinder);

   const int points_no = 6953;

   load_2dpoints(path_points1,points_no, points_view1);
   load_2dpoints(path_points2,points_no, points_view2);

   fundamentalFinder->findFundamental(points_view1, points_view2, F);

   for(int ii = 0; ii < 3; ++ii){
       for(int jj = 0; jj < 3; ++jj){
           std::cout<<F(ii,jj)<<" ";
       }
       std::cout<<std::endl;
   }

   std::ofstream ox(path_fileout.c_str());
   for (int ii = 0; ii < 3; ++ii) {
	   for (int jj = 0; jj < 3; ++jj) {
		   ox << F(ii, jj) << " ";
	   }
	   ox << std::endl;
   }
   ox.close();


   return 0;
}

int printHelp(){
        printf(" usage :\n");
        printf(" exe path_points1 path_points2 fileOut\n");
        return 1;
}

int main(int argc, char **argv){



    if(argc == 4){
		std::string path_points1 = std::string(argv[1]);
		std::string path_points2 = std::string(argv[2]);
		std::string path_fileout = std::string(argv[3]);

        run(path_points1, path_points2, path_fileout);

        return 1;
    }
    else
        return(printHelp());
        
}



