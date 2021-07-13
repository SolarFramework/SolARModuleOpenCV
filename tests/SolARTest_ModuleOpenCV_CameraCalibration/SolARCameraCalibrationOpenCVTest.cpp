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

// ADD COMPONENTS HEADERS HERE, e.g #include "SolarComponent.h"

#include "api/input/devices/ICameraCalibration.h"

#include "core/Log.h"

#include <iostream>
#include <map>

using namespace std;
using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf = org::bcom::xpcf;

void printHelp() {
	std::cout << "Missing parameters" << std::endl;
	std::string readmeFile = std::string("../readme.adoc");
	std::ifstream ifs(readmeFile.c_str());
	if (!ifs) {
		LOG_ERROR("Readme File {} does not exist", readmeFile.c_str());
		return;
	}
	std::string line;
	while (std::getline(ifs, line))
		//  printf("{}", line);
		std::cout << line << std::endl;
	ifs.close();
	return;
}


int calibratio_run(int cameraId) {

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("SolARTest_ModuleOpenCV_CameraCalibration_conf.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file SolARTest_ModuleOpenCV_CameraCalibration_conf.xml")
        return -1;
    }

    std::string calib_config = std::string("../../data/chessboard_config.yml");
    std::ifstream ifs(calib_config.c_str());
    if (!ifs) {
        LOG_ERROR("Calibration config File {} does not exist", calib_config.c_str());
        printHelp();
        return -1;
    }

    std::string calib_output = std::string("../../data/camera_calibration.yml");

    // declare and create components
    LOG_INFO("Start creating components");

    auto cameraCalibration = xpcfComponentManager->resolve<input::devices::ICameraCalibration>();

    SRef<Image> inputImage;

	if (cameraCalibration->setParameters(calib_config))
	{
		cameraCalibration->calibrate(cameraId, calib_output);
	}
	else
	{
		printHelp();
	}

	return 0;
}


int calibratio_run(std::string& video) {

    LOG_ADD_LOG_TO_CONSOLE();

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("SolARTest_ModuleOpenCV_CameraCalibration_conf.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file SolARTest_ModuleOpenCV_CameraCalibration_conf.xml")
        return -1;
    }

    std::string calib_config = std::string("../../data/chessboard_config.yml");
    std::ifstream ifs(calib_config.c_str());
    if (!ifs) {
        LOG_ERROR("Calibration config File {} does not exist", calib_config.c_str());
        printHelp();
        return -1;
    }

    std::string calib_output = std::string("../../data/camera_calibration.yml");

    // declare and create components
    LOG_INFO("Start creating components");

    auto cameraCalibration = xpcfComponentManager->resolve<input::devices::ICameraCalibration>();

    SRef<Image> inputImage;

    if (cameraCalibration->setParameters(calib_config))
	{
		cameraCalibration->calibrate(video, calib_output);
	}
	else
	{
		printHelp();
	}

	return 0;
}
int main(int argc, char* argv[]) {
	LOG_ADD_LOG_TO_CONSOLE();

	if (argc == 2) {

		//  initalizes camera with the given parameter of the program
		std::string cameraArg = std::string(argv[1]);

		//  checks if a video is given in parameters
		if (cameraArg.find("mp4") != std::string::npos || cameraArg.find("mov") != std::string::npos || cameraArg.find("wmv") != std::string::npos || cameraArg.find("avi") != std::string::npos)
		{
			return calibratio_run(cameraArg);
		}
		else
		{  //no video in parameters, then the input camera is used
			return calibratio_run(atoi(argv[1]));
		}
	}
	return calibratio_run(0);


}

