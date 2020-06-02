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
#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"

// ADD COMPONENTS HEADERS HER

#include "api/input/devices/ICamera.h"
#include "api/solver/pose/I2DTransformFinder.h"
#include "api/solver/pose/I2Dto3DTransformDecomposer.h"
#include "core/Log.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

void load_2dpoints(std::string&path_file, int points_no, std::vector<Point2Df>&pt2d){

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
       pt2d[i]  = Point2Df(v[0], v[1]);
    }
  ox.close();
}

int main() {
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    std::string path_points1 = "../../data/pt1_F.txt";
    std::string path_points2 = "../../data/pt2_F.txt";

    try{
        /* instantiate component manager*/
        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if(xpcfComponentManager->load("SolAROpenCVFundamentalMatrixDecomposer_conf.xml")!=org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file SolAROpenCVFundamentalMatrixDecomposer_conf.xml")
            return -1;
        }

        // declare and create components
        LOG_INFO("Start creating components");

        // component creation
        SRef<input::devices::ICamera> camera = xpcfComponentManager->resolve<input::devices::ICamera>();
        SRef<solver::pose::I2DTransformFinder> fundamentalFinder = xpcfComponentManager->resolve<solver::pose::I2DTransformFinder>();
        SRef<solver::pose::I2Dto3DTransformDecomposer> fundamentalDecomposer = xpcfComponentManager->resolve<solver::pose::I2Dto3DTransformDecomposer>();

        /* we need to check that components are well created*/
        if (!camera || !fundamentalFinder || !fundamentalDecomposer)
        {
            LOG_ERROR("One or more component creations have failed");
            return -1;
        }


     // declarations
        Transform2Df                                      F;
        std::vector<Transform3Df>                         poses;
        std::vector<Point2Df>                       points_view1;
        std::vector<Point2Df>                       points_view2;

     // Initialization
       fundamentalDecomposer->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistortionParameters());

       const int nb_points = 6953;
       load_2dpoints(path_points1, nb_points, points_view1);
       load_2dpoints(path_points2, nb_points, points_view2);

       fundamentalFinder->find(points_view1, points_view2, F);
       fundamentalDecomposer->decompose(F,poses);

       for(int k = 0; k <poses.size(); ++k)
           LOG_INFO("pose {} \n {}", k, poses[k].matrix());
    }

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }

    return 0;

}


