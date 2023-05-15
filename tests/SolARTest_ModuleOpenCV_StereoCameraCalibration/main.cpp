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

#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include "core/Log.h"
#include "api/input/devices/IARDevice.h"
#include "api/display/IImageViewer.h"
#include "api/input/devices/IStereoCameraCalibration.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
namespace xpcf  = org::bcom::xpcf;

int main(int argc, char *argv[])
{

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

	try {
		/* instantiate component manager*/
		/* this is needed in dynamic mode */
		SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

		std::string configxml = std::string("SolARTest_ModuleOpenCV_StereoCameraCalibration_conf.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);

		if (xpcfComponentManager->load(configxml.c_str()) != org::bcom::xpcf::_SUCCESS)
		{
			LOG_ERROR("Failed to load the configuration file {}", configxml.c_str());
			return -1;
		}

		// declare and create components
		LOG_INFO("Start creating components");
		auto arDevice = xpcfComponentManager->resolve<input::devices::IARDevice>();
		auto imageLeftViewer = xpcfComponentManager->resolve<display::IImageViewer>("Left");
		auto imageRightViewer = xpcfComponentManager->resolve<display::IImageViewer>("Right");
		auto stereoCalibrator = xpcfComponentManager->resolve<input::devices::IStereoCameraCalibration>();
		LOG_INFO("Components created!");

		if (arDevice->start() == FrameworkReturnCode::_ERROR_)
		{
			LOG_ERROR("Cannot start loader");
			return -1;
		}
		LOG_INFO("Started!");

		// Load camera intrinsics parameters of the left and right camera 
		CameraRigParameters cameraRigParams = arDevice->getCameraParameters();
		CameraParameters camParamsL = cameraRigParams.cameraParams[1];
		CameraParameters camParamsR = cameraRigParams.cameraParams[2];
		
		// collect left and right image
		std::vector<SRef<Image>> imagesL, imagesR;
		LOG_INFO("Loading images");
		while (true) {
			// get data
			std::vector<SRef<Image>> images;
			std::vector<Transform3Df> poses;
			std::chrono::system_clock::time_point timestamp;
			if (arDevice->getData(images, poses, timestamp) != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Error during capture");
				break;
			}
			// get image left and right
			SRef<Image> imageL = images[1];
			SRef<Image> imageR = images[2];
			imagesL.push_back(imageL);
			imagesR.push_back(imageR);
		}
		LOG_INFO("Number of images for calibration: {}", imagesL.size());

		// stereo calibration
		Transform3Df transformation;
		RectificationParameters rectParamsL, rectParamsR;
		stereoCalibrator->calibrate(imagesL, imagesR, camParamsL, camParamsR, 
			transformation, rectParamsL, rectParamsR);

		// add rectification params to camera rig params
		cameraRigParams.transformations[std::make_pair(1, 2)] = transformation;
		cameraRigParams.rectificationParams[std::make_pair(1, 2)] = std::make_pair(rectParamsL, rectParamsR);

		// save to file
		std::string stereoRectFile = "stereo_rectification.json";
		saveToFile(cameraRigParams, stereoRectFile);
		
	}

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }
    return 0;
}
