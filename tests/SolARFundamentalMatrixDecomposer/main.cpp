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

#include "SolARSVDFundamentalMatrixDecomposerOpencv.h"
#include "SolARFundamentalMatrixEstimationOpencv.h"

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
int run(std::string& path_points1,std::string& path_points2, std::string& outPosesFilePath) {
	
	cv::namedWindow("main window",0);
 // declarations
    xpcf::utils::uuids::string_generator              gen;
    SRef<solver::pose::IFundamentalMatrixEstimation>  fundamentalFinder;
    SRef<solver::pose::IFundamentalMatrixDecomposer>  fundamentalDecomposer;
    Transform2Df                                      F;
    CamCalibration                                    K;
    std::vector<SRef<Pose>>                           poses;
    std::vector<SRef<Point2Df>>                       points_view1;
    std::vector<SRef<Point2Df>>                       points_view2;




    // The escape key to exit the sample
    char escape_key = 27;

 // component creation
    xpcf::ComponentFactory::createComponent<SolARFundamentalMatrixEstimationOpencv>(gen(solver::pose::IFundamentalMatrixEstimation::UUID ), fundamentalFinder);
    xpcf::ComponentFactory::createComponent<SolARSVDFundamentalMatrixDecomposerOpencv>(gen(solver::pose::IFundamentalMatrixDecomposer::UUID ), fundamentalDecomposer);


   const int points_no = 6953;
   load_2dpoints(path_points1,points_no, points_view1);
   load_2dpoints(path_points2,points_no, points_view2);

   fillK(K);

   fundamentalFinder->findFundamental(points_view1, points_view2, F);
   fundamentalDecomposer->decompose(F,K,poses);
   std::ofstream ox(outPosesFilePath.c_str());
   for(int k = 0; k <poses.size(); ++k){
       ox<<"--pose: "<<k<<std::endl;
       for(int ii = 0; ii < 4; ++ii){
           for(int jj = 0; jj < 4; ++jj){
               ox<<poses[k]->m_poseTransform(ii,jj)<<" ";
           }
           ox<<std::endl;
       }
       ox<<std::endl<<std::endl;
   }
   ox.close();
   return 0;
}

int printHelp(){
        printf(" usage :\n");
        printf(" exe firstImagePath secondImagePath outPosesFilePath\n");
        return 1;
}

int main(int argc, char **argv){
	if (argc != 4) {
		printHelp();
		return 1;
	}

	std::string firstImagePath = std::string(argv[1]);
	std::string secondImagePath = std::string(argv[2]);
	std::string outPosesFilePath = std::string(argv[3]);

	run(firstImagePath,secondImagePath,outPosesFilePath);
    return 0;
}



