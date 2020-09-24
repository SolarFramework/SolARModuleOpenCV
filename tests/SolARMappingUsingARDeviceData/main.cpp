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
#include "api/display/I2DOverlay.h"
#include "api/display/I3DPointsViewer.h"
#include "api/display/IMatchesOverlay.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/map/IMapper.h"
#include "api/solver/map/IKeyframeSelector.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapFilter.h"
#include "api/solver/map/IBundler.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/loop/ILoopCorrector.h"
#include "api/slam/IBootstrapper.h"
#include "api/slam/IMapping.h"

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

        if(xpcfComponentManager->load("conf_SolARMappingUsingARDeviceData.xml")!=org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file conf_SolARMappingUsingARDeviceData.xml")
            return -1;
        }

        // declare and create components
        LOG_INFO("Start creating components");
		auto arDevice = xpcfComponentManager->resolve<input::devices::IARDevice>();
		auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
		auto overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();
		auto overlay2D = xpcfComponentManager->resolve<display::I2DOverlay>();
		auto viewer3D = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		auto matchesOverlay = xpcfComponentManager->resolve<api::display::IMatchesOverlay>();
		auto pointCloudManager = xpcfComponentManager->resolve<IPointCloudManager>();
		auto keyframesManager = xpcfComponentManager->resolve<IKeyframesManager>();
		auto covisibilityGraph = xpcfComponentManager->resolve<ICovisibilityGraph>();
		auto keyframeRetriever = xpcfComponentManager->resolve<IKeyframeRetriever>();
		auto mapper = xpcfComponentManager->resolve<solver::map::IMapper>();
		auto keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
		auto descriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
		auto matcher = xpcfComponentManager->resolve<features::IDescriptorMatcher>();
		auto keyframeSelector = xpcfComponentManager->resolve<solver::map::IKeyframeSelector>();
		auto triangulator = xpcfComponentManager->resolve<api::solver::map::ITriangulator>();
		auto mapFilter = xpcfComponentManager->resolve<api::solver::map::IMapFilter>();
		auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
		auto matchesFilter = xpcfComponentManager->resolve<features::IMatchesFilter>(); ;
		auto loopDetector = xpcfComponentManager->resolve<loop::ILoopClosureDetector>();
		auto loopCorrector = xpcfComponentManager->resolve<loop::ILoopCorrector>();
		auto mapping = xpcfComponentManager->resolve<slam::IMapping>();
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
		triangulator->setCameraParameters(camParams.intrinsic, camParams.distortion);
		loopDetector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		loopCorrector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		mapping->setCameraParameters(camParams.intrinsic, camParams.distortion);
		LOG_DEBUG("Loaded intrinsics \n{}\n\n{}", camParams.intrinsic, camParams.distortion);

		// Bootstrap
		SRef<Keyframe> keyframe1, keyframe2;
		if (mapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Load map done!");
			keyframesManager->getKeyframe(0, keyframe2);
		}
		else {
			LOG_INFO("Initialization from scratch");
			bool bootstrapOk = false;
			bool initFrame1 = false;
			while (!bootstrapOk) {
				// get data
				std::vector<SRef<Image>> images;
				std::vector<Transform3Df> poses;
				std::chrono::system_clock::time_point timestamp;
				if (arDevice->getData(images, poses, timestamp) != FrameworkReturnCode::_SUCCESS) {
					LOG_ERROR("Error during capture");
					break;
				}
				std::vector<Keypoint> keypoints;
				SRef<Image> image = images[0];
				keypointsDetector->detect(image, keypoints);
				SRef<DescriptorBuffer> descriptors;
				descriptorExtractor->extract(image, keypoints, descriptors);
				SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, descriptors, image, poses[0]);				

				if (!initFrame1) {
					initFrame1 = true;
					keyframe1 = xpcf::utils::make_shared<Keyframe>(frame);
				}
				else {
					// matching
					std::vector<DescriptorMatch> matches;
					matcher->match(keyframe1->getDescriptors(), frame->getDescriptors(), matches);
					matchesFilter->filter(matches, matches, keyframe1->getKeypoints(), frame->getKeypoints());		
					frame->setReferenceKeyframe(keyframe1);
					SRef<Image> imageMatches = image->copy();
					if (matches.size() > 0) {
						matchesOverlay->draw(image, imageMatches, keyframe1->getKeypoints(), frame->getKeypoints(), matches);
					}
					if (imageViewer->display(imageMatches) == SolAR::FrameworkReturnCode::_STOP)
						break;
					if (matches.size() < 50) {						
						keyframe1 = xpcf::utils::make_shared<Keyframe>(frame);
					}
					else if (keyframeSelector->select(frame, matches)) {
						// Triangulate
						std::vector<SRef<CloudPoint>> cloud, filteredCloud;
						triangulator->triangulate(keyframe1->getKeypoints(), frame->getKeypoints(), keyframe1->getDescriptors(), frame->getDescriptors(), matches,
							std::make_pair(0, 1), keyframe1->getPose(), frame->getPose(), cloud);
						mapFilter->filter(keyframe1->getPose(), frame->getPose(), cloud, filteredCloud);
						if (filteredCloud.size() > 50) {
							// add keyframes to keyframes manager
							keyframesManager->addKeyframe(keyframe1);
							keyframe2 = xpcf::utils::make_shared<Keyframe>(frame);
							keyframesManager->addKeyframe(keyframe2);
							keyframe2->setReferenceKeyframe(keyframe1);
							// add intial point cloud to point cloud manager and update visibility map and update covisibility graph
							for (auto const &it : filteredCloud)
								mapper->addCloudPoint(it);
							// add keyframes to retriever
							keyframeRetriever->addKeyframe(keyframe1);
							keyframeRetriever->addKeyframe(keyframe2);
							// apply bundle adjustement 
							double bundleReprojError = bundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);
							bootstrapOk = true;
						}
						else {
							keyframe1 = xpcf::utils::make_shared<Keyframe>(frame);
						}
					}
				}
			}
		}
		LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());

        // Mapping
		std::vector<Transform3Df>   framePoses;
		SRef<Keyframe> refKeyframe = keyframe2;
		while (true)
		{
			// get data
			std::vector<SRef<Image>> images;
			std::vector<Transform3Df> poses;
			std::chrono::system_clock::time_point timestamp;
			if (arDevice->getData(images, poses, timestamp) != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Error during capture");
				break;
			}

			// feature extraction image 0
			std::vector<Keypoint> keypoints;
			SRef<Image> image = images[0];
			keypointsDetector->detect(image, keypoints);
			SRef<DescriptorBuffer> descriptors;
			descriptorExtractor->extract(image, keypoints, descriptors);
			SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, descriptors, image, refKeyframe, poses[0]);
			framePoses.push_back(poses[0]);
			// feature matching to reference keyframe
			std::vector<DescriptorMatch> matches;
			matcher->match(refKeyframe->getDescriptors(), descriptors, matches);
			matchesFilter->filter(matches, matches, refKeyframe->getKeypoints(), keypoints);
			matchesFilter->filter(matches, matches, refKeyframe->getKeypoints(), keypoints, refKeyframe->getPose(), frame->getPose(), camParams.intrinsic);

			// update visibilities of current frame
			std::map<uint32_t, uint32_t> newMapVisibility;
			std::map<uint32_t, uint32_t> refKfVisibility = refKeyframe->getVisibility();
			std::vector<Point2Df> pts2d;
			for (auto &it_match : matches) {
				int idKpFrame = it_match.getIndexInDescriptorB();
				int idKpRefKf = it_match.getIndexInDescriptorA();
				auto itCP = refKfVisibility.find(idKpRefKf);
				if (itCP != refKfVisibility.end()) {
					newMapVisibility[idKpFrame] = itCP->second;
					pts2d.push_back(Point2Df(keypoints[idKpFrame].getX(), keypoints[idKpFrame].getY()));
				}
			}
			frame->addVisibilities(newMapVisibility);
			LOG_INFO("Number of tracked points: {}", newMapVisibility.size());
			// mapping
			SRef<Keyframe> keyframe;
			if (mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
				LOG_INFO("New keyframe id: {}", keyframe->getId());
			}
			// update reference keyframe
			if (keyframe) {
				refKeyframe = keyframe;
			}

			// draw pose
			SRef<Image> displayImage = image->copy();
			overlay3D->draw(frame->getPose(), displayImage);
			overlay2D->drawCircles(pts2d, displayImage);

			// display
			if (imageViewer->display(displayImage) == SolAR::FrameworkReturnCode::_STOP)
				break;

			// get all keyframes and point cloud
			std::vector<Transform3Df> keyframePoses;
			std::vector<SRef<Keyframe>> allKeyframes;
			keyframesManager->getAllKeyframes(allKeyframes);
			for (auto const &it : allKeyframes)
				keyframePoses.push_back(it->getPose());
			std::vector<SRef<CloudPoint>> pointCloud;
			pointCloudManager->getAllPoints(pointCloud);
			// display point cloud 
			if (viewer3D->display(pointCloud, frame->getPose(), keyframePoses, framePoses) == FrameworkReturnCode::_STOP)
				break;
        }
    }

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }
    return 0;
}
