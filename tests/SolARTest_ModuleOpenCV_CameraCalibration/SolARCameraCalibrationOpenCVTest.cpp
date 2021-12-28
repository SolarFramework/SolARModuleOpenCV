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

#include "api/input/devices/ICamera.h"
#include "api/display/IImageViewer.h"
#include "api/input/devices/ICameraCalibration.h"
#include "core/Log.h"
#include <iostream>
#include <map>
#include <string>

using namespace std;
using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf = org::bcom::xpcf;

int main(int argc, char* argv[]) {
	SRef<xpcf::IComponentManager> xpcfComponentManager;
	LOG_ADD_LOG_TO_CONSOLE();
	try {
		xpcfComponentManager = xpcf::getComponentManagerInstance();
		std::string configxml = std::string("SolARTest_ModuleOpenCV_CameraCalibration_conf.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);
		if (xpcfComponentManager->load(configxml.c_str()) != org::bcom::xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file {}", configxml.c_str())
				return -1;
		}
		LOG_INFO("Start creating components");
		auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
		auto viewer = xpcfComponentManager->resolve<display::IImageViewer>();
		auto calibrator = xpcfComponentManager->resolve<input::devices::ICameraCalibration>();
		LOG_INFO("Loaded all components");
		// set resolution
		Sizei resolution;
		resolution.width = calibrator->bindTo<xpcf::IConfigurable>()->getProperty("image_width")->getIntegerValue();
		resolution.height = calibrator->bindTo<xpcf::IConfigurable>()->getProperty("image_height")->getIntegerValue();		
		camera->setResolution(resolution);
		// start camera
		if (camera->start() != FrameworkReturnCode::_SUCCESS) {
			LOG_ERROR("Cannot open camera");
			return -1;
		}
		// get data
		int nbMaxImages = 500;
		std::vector<SRef<Image>> images;
		while (true) {
			SRef<Image> image;
			if (camera->getNextImage(image) != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Error during capture");
				return -1;
			}
			if (viewer->display(image) != FrameworkReturnCode::_SUCCESS) break;
			images.push_back(image);
			if (images.size() >= nbMaxImages)
				break;
			std::cout << "Number of captured images: " << images.size() << "\r";
		}
		// calibrate
		CameraParameters camParams;
		camParams.id = 0;
		camParams.name = "Camera";
		camParams.type = CameraType::RGB;
		calibrator->calibrate(images, camParams);
		saveToFile(camParams, "camera_calibration.json");
	}
	catch (xpcf::Exception e)
	{
		LOG_ERROR("The following exception has been catch : {}", e.what());
		return -1;
	}

	return 0;
}

