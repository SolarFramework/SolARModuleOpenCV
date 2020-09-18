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
#include "api/display/I3DOverlay.h"
#include "api/display/I3DPointsViewer.h"

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

        if(xpcfComponentManager->load("conf_SolARDeviceDataLoaderTest.xml")!=org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file conf_SolARDeviceDataLoaderTest.xml")
            return -1;
        }

        // declare and create components
        LOG_INFO("Start creating components");
		auto arDevice = xpcfComponentManager->resolve<input::devices::IARDevice>();
		auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
		auto overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();
		auto viewer3D = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		LOG_INFO("Components created!");

		LOG_INFO("Start AR device loader");
		// Connect remotely to the HoloLens streaming app
		if (arDevice->start() == FrameworkReturnCode::_ERROR_)
		{
			LOG_ERROR("Cannot start loader");
			return -1;
		}
		LOG_INFO("Started!");

		// Load camera intrinsics parameters
		CameraParameters camParams;
		camParams = arDevice->getParameters(0);
		overlay3D->setCameraParameters(camParams.intrinsic, camParams.distortion);
		LOG_INFO("Loaded intrinsics \n{}\n\n{}", camParams.intrinsic, camParams.distortion);

        // Display images and poses
		std::vector<SRef<CloudPoint>> pointCloud;
		pointCloud.push_back(xpcf::utils::make_shared<CloudPoint>(0, 0, 0));
		pointCloud.push_back(xpcf::utils::make_shared<CloudPoint>(1, 0, 0));
		pointCloud.push_back(xpcf::utils::make_shared<CloudPoint>(0, 1, 0));
		pointCloud.push_back(xpcf::utils::make_shared<CloudPoint>(0, 0, 1));
		std::vector<std::vector<Transform3Df>> keyframePoses(arDevice->getNbCameras());
		while (true)
		{
			// get data
			std::vector<SRef<Image>> frames;
			std::vector<Transform3Df> poses;
			std::chrono::system_clock::time_point timestamp;
			if (arDevice->getData(frames, poses, timestamp) != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Error during capture");
				break;
			}

			// draw pose
			overlay3D->draw(poses[0], frames[0]);

			// display
			if (imageViewer->display(frames[0]) == SolAR::FrameworkReturnCode::_STOP)
				break;

			if (viewer3D->display(pointCloud, poses[0], keyframePoses[0], {}, {}, {}) == SolAR::FrameworkReturnCode::_STOP)
				break;

			keyframePoses[0].push_back(poses[0]);
        }
    }

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }
    return 0;
}
