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
#include <xpcf/api/IComponentManager.h>
#include <api/input/devices/ICamera.h>
#include <api/display/I2DOverlay.h>
#include <api/display/IImageViewer.h>
#include <api/input/files/IWorldGraphLoader.h>
#include <api/features/I2DTrackablesDetector.h>
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;
using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

int main(int argc, char* argv[])
{
#if NDEBUG
	boost::log::core::get()->set_logging_enabled(false);
#endif
	LOG_ADD_LOG_TO_CONSOLE();
	SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if (xpcfComponentManager->load("SolARTest_ModuleOpenCV_QRCodesDetection_conf.xml") != org::bcom::xpcf::_SUCCESS)
	{
        std::cerr << "Failed to load the configuration file SolARTest_ModuleOpenCV_QRCodesDetection_conf.xml" << std::endl;
		return -1;
	}
	auto camera = xpcfComponentManager->resolve<SolAR::api::input::devices::ICamera>();
	auto viewer = xpcfComponentManager->resolve<SolAR::api::display::IImageViewer>();
    auto trackablesDetector = xpcfComponentManager->resolve<SolAR::api::features::I2DTrackablesDetector>();
	auto worldGraphLoader = xpcfComponentManager->resolve<SolAR::api::input::files::IWorldGraphLoader>();
    auto overlay2D = xpcfComponentManager->resolve<SolAR::api::display::I2DOverlay>();

	// load trackables of the world graph
	std::vector<SRef<Trackable>> trackables;
	if (worldGraphLoader->load(trackables) == FrameworkReturnCode::_ERROR_) {
		LOG_ERROR("Error during load the world graph file");
		return -1;
	}
	// get fiducial markers
	std::vector<SRef<Trackable>> qrCodeMarkers;
	for (const auto& trackable : trackables)
		if (trackable->getType() == QRCODE_MARKER)
			qrCodeMarkers.push_back(trackable);
	LOG_INFO("Number of used QR codes: {}", qrCodeMarkers.size());
    // set QR codes to detector
    trackablesDetector->setTrackables(qrCodeMarkers);

	// start camera
	if (camera->start() != FrameworkReturnCode::_SUCCESS) {
		LOG_ERROR("Cannot open camera");
		return -1;
	}

	// process
	while (true) {
		SRef<Image> image;
		if (camera->getNextImage(image) != FrameworkReturnCode::_SUCCESS) {
			LOG_ERROR("Error during capture image");
			return -1;
		}
        std::vector<std::vector<Point2Df>> corners;
        if (trackablesDetector->detect(image, corners) == FrameworkReturnCode::_SUCCESS) {
            overlay2D->drawContours(corners, image);
        }

		if (viewer->display(image) != FrameworkReturnCode::_SUCCESS)
			break;

	}
	return 0;
}
