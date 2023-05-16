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
#include "api/features/IKeypointDetector.h"
#include "api/geom/IUndistortPoints.h"
#include "api/geom/I2DPointsRectification.h"
#include "api/image/IImageRectification.h"
#include "api/display/I2DOverlay.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
namespace xpcf  = org::bcom::xpcf;

// index camera 1: left
#define INDEX_CAMERA 1

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

        std::string configxml = std::string("SolARTest_ModuleOpenCV_StereoKeypointRectification_conf.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);

        if(xpcfComponentManager->load(configxml.c_str())!=org::bcom::xpcf::_SUCCESS)
        {
			LOG_ERROR("Failed to load the configuration file {}", configxml.c_str());
            return -1;
        }

        // declare and create components
        LOG_INFO("Start creating components");
		auto arDevice = xpcfComponentManager->resolve<input::devices::IARDevice>();
		auto keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
		auto undistortKeypoints = xpcfComponentManager->resolve<api::geom::IUndistortPoints>();
		auto stereo2DPointsRectificator = xpcfComponentManager->resolve<geom::I2DPointsRectification>();
		auto stereoImageRectificator = xpcfComponentManager->resolve<image::IImageRectification>();
		auto overlay2D = xpcfComponentManager->resolve<display::I2DOverlay>();
		auto imageViewerOrigin = xpcfComponentManager->resolve<display::IImageViewer>("Origin");
		auto imageViewerRect = xpcfComponentManager->resolve<display::IImageViewer>("Rectified");
		LOG_INFO("Components created!");		

		if (arDevice->start() == FrameworkReturnCode::_ERROR_)
		{
			LOG_ERROR("Cannot start loader");
			return -1;
		}
		LOG_INFO("Started!");

		// get rectification parameters
		CameraRigParameters camRigParams = arDevice->getCameraParameters();
		if (camRigParams.rectificationParams.find(std::make_pair(1, 2)) == camRigParams.rectificationParams.end()) {
			LOG_ERROR("Cannot find rectification parameters between camera 1 and 2");
			return -1;
		}
		RectificationParameters rectParamsL = camRigParams.rectificationParams[std::make_pair(1, 2)].first;
		RectificationParameters rectParamsR = camRigParams.rectificationParams[std::make_pair(1, 2)].second;
		CameraParameters camParams = camRigParams.cameraParams[INDEX_CAMERA];
		
		while (true) {
			// get data
			std::vector<SRef<Image>> images;
			std::vector<Transform3Df> poses;
			std::chrono::system_clock::time_point timestamp;
			if (arDevice->getData(images, poses, timestamp) != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Error during capture");
				break;
			}
			// get image 
			SRef<Image> image = images[INDEX_CAMERA];

			// keypoint detection
			std::vector<Keypoint> keypoints, undistortedKeypoints;
			keypointsDetector->detect(image, keypoints);
			undistortKeypoints->undistort(keypoints, camParams, undistortedKeypoints);
			
			// rectify image
			SRef<Image> imageRectified;
			stereoImageRectificator->rectify(image, camParams, rectParamsL, imageRectified);

			// rectify points
			std::vector<Keypoint> undistortedKeypointsRectified;
			stereo2DPointsRectificator->rectify(undistortedKeypoints, camParams, rectParamsL, undistortedKeypointsRectified);

			// draw circle
			overlay2D->drawCircles(undistortedKeypoints, image);
			overlay2D->drawCircles(undistortedKeypointsRectified, imageRectified);

			// display
			imageViewerOrigin->display(image);
			while (imageViewerRect->display(imageRectified) == FrameworkReturnCode::_SUCCESS);
		}
    }

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }
    return 0;
}
