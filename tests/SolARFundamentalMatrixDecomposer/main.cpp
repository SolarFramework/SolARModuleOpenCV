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
#include <fstream>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"

#include "SolARHomographyEstimationOpencv.h"
#include "SolARSVDFundamentalMatrixDecomposerOpencv.h"

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

void fillDist(CamDistortion&dist){
    dist(0) = 0.0;
    dist(1) = 0.0;
    dist(2) = 0.0;
    dist(3) = 0.0;
}

void load_2dpoints(std::string&path_file, int points_no, std::vector<SRef<Point2Df>>&pt2d){

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
       pt2d[i]  = xpcf::utils::make_shared<Point2Df>(v[0], v[1]);
    }
  ox.close();
}
int run(std::string& path_points1,std::string& path_points2, std::string& outPosesFilePath) {
	
 // declarations
    Transform2Df                                      F;
    CamCalibration                                    K;
    CamDistortion                                     dist;
    std::vector<Transform3Df>                         poses;
    std::vector<SRef<Point2Df>>                       points_view1;
    std::vector<SRef<Point2Df>>                       points_view2;




    // The escape key to exit the sample
    char escape_key = 27;

 // component creation
    auto fundamentalFinder =xpcf::ComponentFactory::createInstance<SolARHomographyEstimationOpencv>()->bindTo<solver::pose::I2DTransformFinder>();
    auto fundamentalDecomposer =xpcf::ComponentFactory::createInstance<SolARSVDFundamentalMatrixDecomposerOpencv>()->bindTo<api::solver::pose::I2DTO3DTransformDecomposer>();


   const int points_no = 6953;
   load_2dpoints(path_points1,points_no, points_view1);
   load_2dpoints(path_points2,points_no, points_view2);

   fillK(K);
   fillDist(dist);

   fundamentalFinder->find(points_view1, points_view2, F);
   fundamentalDecomposer->decompose(F,K,dist,poses);

   for(int k = 0; k <poses.size(); ++k){
       std::cout<<"--pose: "<<k<<std::endl;
       for(int ii = 0; ii < 4; ++ii){
           for(int jj = 0; jj < 4; ++jj){
               std::cout<<poses[k](ii,jj)<<" ";
           }
           std::cout<<std::endl;
       }
       std::cout<<std::endl<<std::endl;
   }
   return 0;
}

int printHelp(){
        printf(" usage :\n");
        printf(" exe firstImagePointsPath secondImagePointsPath outPosesFilePath\n");
        return 1;
}

int main(int argc, char **argv){
	if (argc != 4) {
		printHelp();
		return 1;
	}

    std::string firstImagePointsPath = std::string(argv[1]);
    std::string secondImagePointsPath = std::string(argv[2]);
	std::string outPosesFilePath = std::string(argv[3]);

    run(firstImagePointsPath,secondImagePointsPath,outPosesFilePath);
    return 0;
}



