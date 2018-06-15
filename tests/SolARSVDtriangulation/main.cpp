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

#include "SolARSVDTriangulationOpencv.h"


using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;

namespace xpcf  = org::bcom::xpcf;

bool load_2dpoints(std::string&path_file, int points_no, std::vector<SRef<Point2Df>>&pt2d){
    std::ifstream ox(path_file);
    if (!ox)
        return false;

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
  return true;
}


bool load_pose(std::string &path_file, Transform3Df &P){
    std::ifstream ox(path_file);
    if (!ox)
        return false;

    float v;
    std::string dummy;
    for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 4; ++j){

            ox>>dummy;

            P(i,j) = std::stof(dummy);
        }
    }

    P(3,0) = 0.0;
    P(3,1) = 0.0;
    P(3,2) = 0.0;
    P(3,3) = 1.0;

    for(int i = 0; i < 4; ++i){
        for(int j = 0; j < 4; ++j){
            std::cout<<P(i,j)<<" ";
        }
        std::cout<<std::endl;
    }
   ox.close();

   return true;
}

bool load_camMatrix(std::string &path_file, CamCalibration&intrinsic){
    std::ifstream ox(path_file);
    if (!ox)
        return false;

    std::string dummy;
    float v;
    for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 3; ++j){
            ox>>dummy;
            v = std::stof(dummy);
            intrinsic(i,j) = v;
            std::cout<<intrinsic(i,j)<<" ";
        }
        std::cout<<std::endl;
    }
    ox.close();

    return true;
}

bool load_distorsion(std::string&path_file, CamDistortion&dist){
     std::ifstream ox(path_file);
     if (!ox)
         return false;

    std::string dummy;
    float v;
    for(int i = 0; i < 4; ++i){
        ox>>dummy;
        v = std::stof(dummy);
        dist(i,0) = v;
    }
    dist(4,0) = 0.0;
    std::cout<<dist<<std::endl;

    return true;
}

void help(){
    std::cout << "\n\n";
    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
    std::cout << "Something went wrong with input files \n";
    std::cout << "please refer to README.adoc in the project directory \n";
    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n";
    exit(-1);
}

int run()
{

 // declarations
    xpcf::utils::uuids::string_generator    gen;


 // component creation
  
    auto mapper = xpcf::ComponentFactory::createInstance<SolARSVDTriangulationOpencv>()->bindTo<solver::map::ITriangulator>();




   std::string path_pt1 = "./pt1_F.txt";
   std::string path_pt2 = "./pt2_F.txt";
   std::string path_pose1 = "./pose_1.txt";
   std::string path_pose2 = "./pose_2.txt";
   std::string path_intrinsic = "./K.txt";
   std::string path_distorsion = "./dist.txt";

   std::cout<<" wthat's happenning here.."<<std::endl;

   std::vector<SRef<Point2Df>>pt2d_1;
   std::vector<SRef<Point2Df>>pt2d_2;
   std::vector<SRef<Point3Df>>pt3d;

   Transform3Df pose_1;
   Transform3Df pose_2;
   CamCalibration K;
   CamDistortion dist;

      const int points_no = 6954;
      std::cout<<"->load points 2d view 1: "<<std::endl;
      if(load_2dpoints(path_pt1, points_no, pt2d_1)==false){
          help();
      };

      std::cout<<"->load points 2d view 2: "<<std::endl;
      if(load_2dpoints(path_pt2, points_no, pt2d_2)==false){
          help();
      };;

	  std::cout << "-> load Pose view 1: " << std::endl;
      if(load_pose(path_pose1, pose_1)==false){
          help();
      };

	  std::cout << "-> load Pose view 2: " << std::endl;
      if(load_pose(path_pose2, pose_2)==false){
          help();
      };

     std::cout<<"->load K: "<<std::endl;
      if(load_camMatrix(path_intrinsic,K)==false){
          help();
      };

      std::cout<<"->load dist: "<<std::endl;
      if(load_distorsion(path_distorsion,dist)==false){
          help();
      };

      mapper->triangulate(pt2d_1, pt2d_2,pose_1,pose_2,K,dist,pt3d);


      std::ofstream log_cloud("./solar_cloud.txt");
      log_cloud<<pt3d.size()<<std::endl;
      for(int k = 0; k < pt3d.size(); ++k){
          log_cloud<<pt3d[k]->getX()<<" "<<pt3d[k]->getY()<<" "<<pt3d[k]->getZ()<<std::endl;
      }
      log_cloud.close();

	  return(0);
}

int main(){
	run();
}
