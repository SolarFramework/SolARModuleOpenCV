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
#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"

// ADD COMPONENTS HEADERS HERE

#include "SolARModuleOpencv_traits.h"
#include "api/solver/map/ITriangulator.h"
#include "api/input/devices/ICamera.h"

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

    P = P.inverse();

    LOG_INFO("pose =\n{}", P.matrix());
   ox.close();

   return true;
}

bool create_matches(std::vector<SRef<Point2Df>>&pt2d_1, std::vector<SRef<Point2Df>>&pt2d_2, std::vector<DescriptorMatch>&matches)
{
    int min_size = std::min(pt2d_1.size(), pt2d_2.size());
    for (int i = 0; i < min_size; i++)
    {
        matches.push_back(DescriptorMatch(i,i,1.0));
    }
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

int main()
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    std::string path_pt1 = "../data/pt1_F.txt";
    std::string path_pt2 = "../data/pt2_F.txt";
    std::string path_pose1 = "../data/pose_1.txt";
    std::string path_pose2 = "../data/pose_2.txt";

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("conf_Triangulation.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_Triangulation.xml")
        return -1;
    }

    // declare and create components
    LOG_INFO("Start creating components");

    // component creation
    SRef<input::devices::ICamera> camera = xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    SRef<solver::map::ITriangulator> mapper = xpcfComponentManager->create<SolARSVDTriangulationOpencv>()->bindTo<solver::map::ITriangulator>();

    std::vector<SRef<Point2Df>   >pt2d_1;
    std::vector<SRef<Point2Df>>  pt2d_2;
    std::vector<SRef<CloudPoint>>  pt3d;
    std::vector<DescriptorMatch> matches;
    std::pair<int,int> working_views = {0,1};
    Transform3Df pose_1;
    Transform3Df pose_2;

    const int points_no = 6954;
    LOG_INFO("load points 2d view 1");
    if(!load_2dpoints(path_pt1, points_no, pt2d_1)){
      help();
    };

    LOG_INFO("load points 2d view 2");
    if(!load_2dpoints(path_pt2, points_no, pt2d_2))
      help();

    LOG_INFO("create matches");
    if(!create_matches(pt2d_1, pt2d_2, matches))
      help();

    LOG_INFO("load Pose view 1");
    if(!load_pose(path_pose1, pose_1))
      help();

    LOG_INFO("load Pose view 2");
    if(!load_pose(path_pose2, pose_2))
      help();

    mapper->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    mapper->triangulate(pt2d_1, pt2d_2, matches, working_views, pose_1, pose_2, pt3d);

    std::ofstream log_cloud("./solar_cloud.txt");
    log_cloud<<pt3d.size()<<std::endl;
    for(int k = 0; k < pt3d.size(); ++k){
      log_cloud<<pt3d[k]->getX()<<" "<<pt3d[k]->getY()<<" "<<pt3d[k]->getZ()<<std::endl;
    }
    log_cloud.close();

    return(0);
    }
