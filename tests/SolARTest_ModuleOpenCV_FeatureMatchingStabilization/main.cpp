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

#include "xpcf/xpcf.h"

#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/display/IImageViewer.h"
#include "api/display/IMatchesOverlay.h"
#include "api/display/I2DOverlay.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IMatchesFilter.h"
#include "core/Log.h"

#include <iostream>
#include <boost/log/core.hpp>
#include <string>
#include <vector>

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int main(int argc,char** argv)
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("SolARTest_ModuleOpenCV_FeatureMatchingStabilization_conf.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file SolARTest_ModuleOpenCV_FeatureMatchingStabilization_conf.xml")
        return -1;
    }

    // declare and create components
    LOG_INFO("Start creating components");
	auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
    auto detector = xpcfComponentManager->resolve<features::IKeypointDetector>();
    auto extractor = xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
    auto matcher = xpcfComponentManager->resolve<features::IDescriptorMatcher>();
    auto matchesFilter = xpcfComponentManager->resolve<features::IMatchesFilter>();
    auto overlayMatches = xpcfComponentManager->resolve<display::IMatchesOverlay>();
    auto overlay2D = xpcfComponentManager->resolve<display::I2DOverlay>();
    auto viewer = xpcfComponentManager->resolve<display::IImageViewer>();
	LOG_INFO("All components created!");

	// Open camera
	if (camera->start() != FrameworkReturnCode::_SUCCESS) {
		LOG_ERROR("Camera cannot start");
		return -1;
	}

	// check feature matching stabilization
	SRef<Frame> frame1;
	bool bInitFirstFrame(false);
	for (;;) {
		SRef<Image> image, imageDisplay;
		if (camera->getNextImage(image) != FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Error load image");
			return 0;
		}
		imageDisplay = image->copy();

		std::vector<Keypoint>           keypoints;
		SRef<DescriptorBuffer>          descriptors;
		std::vector<DescriptorMatch>    matches;
		// feature detection
		detector->detect(image, keypoints);
		// feature extraction
		extractor->extract(image, keypoints, descriptors);
		// make a frame
		SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, descriptors, image);
		
		if (!bInitFirstFrame) {
			LOG_INFO("Number of keypoints of the first frame: {}", keypoints.size());
			frame1 = frame;
			bInitFirstFrame = true;
			continue;
		}
		LOG_INFO("Number of keypoints: {}", keypoints.size());
		// feature matching		
		//matcher->match(descriptors, frame1->getDescriptors(), matches);
		matcher->matchInRegion(frame, frame1, matches);
		LOG_INFO("Number of matches: {}", matches.size());		
		// matches filter
		matchesFilter->filter(matches, matches, keypoints, frame1->getKeypoints());
		LOG_INFO("Number of filtered matches: {}\n", matches.size());
		if (matches.size() < 600) {
			frame1 = frame;
			continue;
		}
		// draw keypoints and matches		
		overlayMatches->draw(image, imageDisplay, keypoints, frame1->getKeypoints(), matches);
		overlay2D->drawCircles(keypoints, imageDisplay);

		// display
		if (viewer->display(imageDisplay) == FrameworkReturnCode::_STOP)
			break;
	}

    return 0;
}





