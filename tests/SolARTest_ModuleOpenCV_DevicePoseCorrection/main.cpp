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
#include "api/input/files/ITrackableLoader.h"
#include "api/solver/pose/ITrackablePose.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DOverlay.h"
#include "api/display/I3DPointsViewer.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
namespace xpcf = org::bcom::xpcf;

#define INDEX_USE_CAMERA 0

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

        if (xpcfComponentManager->load("SolARTest_ModuleOpenCV_DevicePoseCorrection_conf.xml") != org::bcom::xpcf::_SUCCESS)
		{
            LOG_ERROR("Failed to load the configuration file SolARTest_ModuleOpenCV_DevicePoseCorrection_conf.xml")
				return -1;
		}

		// declare and create components
		LOG_INFO("Start creating components");
		auto arDevice = xpcfComponentManager->resolve<input::devices::IARDevice>();
        auto fiducialMarkerPoseEstimator = xpcfComponentManager->resolve<solver::pose::ITrackablePose>();
        auto trackableLoader  = xpcfComponentManager->resolve<input::files::ITrackableLoader>();		
		auto viewer3D = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
		auto overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();
		LOG_INFO("Components created!");
        LOG_INFO("Setting marker!");
        SRef<Trackable> trackable;
        if (trackableLoader->loadTrackable(trackable) != FrameworkReturnCode::_SUCCESS)
        {
            LOG_ERROR("cannot load fiducial marker");
            return -1;
        }
        else
        {
            if (fiducialMarkerPoseEstimator->setTrackable(trackable)!= FrameworkReturnCode::_SUCCESS)
            {
                LOG_ERROR("cannot set fiducial marker as a trackable for fiducial marker pose estimator");
                return -1;
            }
        }

		LOG_INFO("Start AR device loader");
		// Connect remotely to the HoloLens streaming app
		if (arDevice->start() == FrameworkReturnCode::_ERROR_)
		{
			LOG_ERROR("Cannot start loader");
			return -1;
		}
		LOG_INFO("Started!");

		// set calibration matrix for components
		CameraRigParameters camRigParams = arDevice->getCameraParameters();
		if (camRigParams.cameraParams.find(INDEX_USE_CAMERA) == camRigParams.cameraParams.end()) {
			LOG_ERROR("Cannot load parameters of camera {}", INDEX_USE_CAMERA);
			return -1;
		}
        CameraParameters camParams = camRigParams.cameraParams[INDEX_USE_CAMERA];

		// Display images and poses
		std::vector<SRef<CloudPoint>> pointCloud;
		pointCloud.push_back(xpcf::utils::make_shared<CloudPoint>(0, 0, 0));
		pointCloud.push_back(xpcf::utils::make_shared<CloudPoint>(1, 0, 0));
		pointCloud.push_back(xpcf::utils::make_shared<CloudPoint>(0, 1, 0));
		pointCloud.push_back(xpcf::utils::make_shared<CloudPoint>(0, 0, 1));
		std::vector<Transform3Df> keyframePoses;
		Transform3Df T_M_W = Transform3Df::Identity();
		bool isStop = false;
		bool isFoundTransform = false;
		while (!isStop)
		{
			// get data
			std::vector<SRef<Image>> images;
			std::vector<Transform3Df> poses;
			std::chrono::system_clock::time_point timestamp;
			if (arDevice->getData(images, poses, timestamp) != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Error during capture");
				break;
			}

			SRef<Image> image = images[INDEX_USE_CAMERA];
			Transform3Df pose = poses[INDEX_USE_CAMERA];

			// find T_W_M
			if (!isFoundTransform) {
				Transform3Df T_M_C;
                if (fiducialMarkerPoseEstimator->estimate(image, camParams, T_M_C) == FrameworkReturnCode::_SUCCESS) {
					T_M_W = T_M_C * pose.inverse();
					isFoundTransform = true;
				}
			}

			// correct pose
			pose = T_M_W * pose;

			// draw pose and display
            overlay3D->draw(pose, camParams, image);
			if (imageViewer->display(image) == SolAR::FrameworkReturnCode::_STOP)
				isStop = true;

			// display 3D camera poses of camera 0
			if (isFoundTransform) {
				keyframePoses.push_back(pose);
				if (viewer3D->display(pointCloud, pose, keyframePoses, {}, {}, {}) == SolAR::FrameworkReturnCode::_STOP)
					break;
			}
		}
	}

	catch (xpcf::Exception e)
	{
		LOG_ERROR("The following exception has been catch : {}", e.what());
		return -1;
	}
	return 0;
}
