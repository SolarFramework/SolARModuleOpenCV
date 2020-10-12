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
#include "api/solver/map/IMapFilter.h"
#include "api/solver/map/IBundler.h"
#include "api/geom/IProject.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/loop/ILoopCorrector.h"
#include "api/slam/IBootstrapper.h"
#include "api/slam/IMapping.h"
#include "api/solver/pose/IFiducialMarkerPose.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
namespace xpcf  = org::bcom::xpcf;

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
		auto overlay2DGreen = xpcfComponentManager->resolve<display::I2DOverlay>("Green");
		auto overlay2DRed = xpcfComponentManager->resolve<display::I2DOverlay>("Red");
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
		auto projector = xpcfComponentManager->resolve<api::geom::IProject>();
		auto mapFilter = xpcfComponentManager->resolve<api::solver::map::IMapFilter>();
		auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>("BundleFixedKeyframes");
		auto matchesFilter = xpcfComponentManager->resolve<features::IMatchesFilter>();
		auto loopDetector = xpcfComponentManager->resolve<loop::ILoopClosureDetector>();
		auto loopCorrector = xpcfComponentManager->resolve<loop::ILoopCorrector>();
		auto bootstrapper = xpcfComponentManager->resolve<slam::IBootstrapper>();
		auto mapping = xpcfComponentManager->resolve<slam::IMapping>();
		auto fiducialMarkerPoseEstimator = xpcfComponentManager->resolve<solver::pose::IFiducialMarkerPose>();
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
		loopDetector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		loopCorrector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		bootstrapper->setCameraParameters(camParams.intrinsic, camParams.distortion);
		mapping->setCameraParameters(camParams.intrinsic, camParams.distortion);
		fiducialMarkerPoseEstimator->setCameraParameters(camParams.intrinsic, camParams.distortion);
		projector->setCameraParameters(camParams.intrinsic, camParams.distortion);
		LOG_DEBUG("Loaded intrinsics \n{}\n\n{}", camParams.intrinsic, camParams.distortion);

		// get properties
		float minWeightNeighbor = mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
		float reprojErrorThreshold = mapper->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();

		// Correct pose and Bootstrap
		Transform3Df T_M_W = Transform3Df::Identity();
		bool isFoundTransform = false;
		bool bootstrapOk = false;
		while (!bootstrapOk) {
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
				if (imageViewer->display(image) == SolAR::FrameworkReturnCode::_STOP)
					exit(0);
				Transform3Df T_M_C;
				if (fiducialMarkerPoseEstimator->estimate(image, T_M_C) == FrameworkReturnCode::_SUCCESS) {
					T_M_W = T_M_C * pose.inverse();
					isFoundTransform = true;
				}
				else
					continue;
			}				
			// correct pose
			pose = T_M_W * pose;
			// do bootstrap
			SRef<Image> view;
			if (bootstrapper->process(image, view, pose) == FrameworkReturnCode::_SUCCESS) {
				// apply bundle adjustement 
				double bundleReprojError = bundler->bundleAdjustment(camParams.intrinsic, camParams.distortion);
				bootstrapOk = true;
			}
			overlay3D->draw(pose, view);
			if (imageViewer->display(view) == SolAR::FrameworkReturnCode::_STOP)
				exit(0);
		}
		LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());
		
		// update local point cloud
		std::vector<SRef<CloudPoint>> localMap;
		auto updateLocalMap=[&](const SRef<Keyframe> &keyframe){
			localMap.clear();			
			mapper->getLocalPointCloud(keyframe, minWeightNeighbor, localMap);
		};

        // Mapping
		std::vector<Transform3Df>   framePoses;
		SRef<Keyframe> refKeyframe; 
		keyframesManager->getKeyframe(1, refKeyframe);
		updateLocalMap(refKeyframe);
		int countNewKeyframes(0);
		while (true)
		{
			LOG_INFO("==========================================");
			LOG_INFO("Ref keyframe id: {}", refKeyframe->getId());
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
			// correct pose
			pose = T_M_W * pose;

			// feature extraction image
			std::vector<Keypoint> keypoints;
			keypointsDetector->detect(image, keypoints);
			SRef<DescriptorBuffer> descriptors;
			descriptorExtractor->extract(image, keypoints, descriptors);
			SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, descriptors, image, refKeyframe, pose);
			framePoses.push_back(pose);

			// feature matching to reference keyframe			
			std::vector<DescriptorMatch> matches;
			matcher->match(refKeyframe->getDescriptors(), descriptors, matches);
			matchesFilter->filter(matches, matches, refKeyframe->getKeypoints(), keypoints);
			float maxMatchDistance = -FLT_MAX;
			for (const auto &it : matches) {
				float score = it.getMatchingScore();
				if (score > maxMatchDistance)
					maxMatchDistance = score;
			}

			// Find map visibilities of the current frame
			std::map<uint32_t, uint32_t> refKfVisibility = refKeyframe->getVisibility();
			std::map<uint32_t, uint32_t> newMapVisibility;	// map visibilities			
			std::vector<Point2Df> pts2d;					// positions of keypoints to visualize
			std::vector<SRef<CloudPoint>> refCPSeen;		// CP visibilities of ref keyframe are seen by the current frame
			std::set<uint32_t> idxCPSeen;					// Index of CP seen
			std::vector<std::pair<uint32_t, uint32_t>> idxKp_idxCP;
			for (auto &it_match : matches) {
				int idKpFrame = it_match.getIndexInDescriptorB();
				int idKpRefKf = it_match.getIndexInDescriptorA();
				auto itCP = refKfVisibility.find(idKpRefKf);
				if (itCP != refKfVisibility.end()) {
					uint32_t idCP = itCP->second;
					pts2d.push_back(Point2Df(keypoints[idKpFrame].getX(), keypoints[idKpFrame].getY()));					
					idxCPSeen.insert(idCP);
					SRef<CloudPoint> cp;
					pointCloudManager->getPoint(idCP, cp);
					refCPSeen.push_back(cp);
					idxKp_idxCP.push_back(std::make_pair(idKpFrame, idCP));
				}
			}			

			// Project ref cloud point seen to current frame to define inliers/outliers
			std::vector< Point2Df > refCPSeenProj;
			int countInliers = 0;
			projector->project(refCPSeen, refCPSeenProj, pose);
			std::vector<Point2Df> pts2d_inliers, pts2d_outliers;
			for (int i = 0; i < refCPSeenProj.size(); ++i) {
				float dis = (pts2d[i] - refCPSeenProj[i]).norm();
				if (dis < reprojErrorThreshold) {
					refCPSeen[i]->updateConfidence(true);
					countInliers++;
					newMapVisibility[idxKp_idxCP[i].first] = idxKp_idxCP[i].second;
					pts2d_inliers.push_back(pts2d[i]);
				}
				else {
					refCPSeen[i]->updateConfidence(false);
					pts2d_outliers.push_back(pts2d[i]);
				}
			}
			LOG_INFO("Number of inliers / outliers: {} / {}", countInliers, pts2d.size() - countInliers);

			// Find more visibilities by projecting the rest of local map
			//  projection points
			std::vector<SRef<CloudPoint>> localMapUnseen;
			for (auto &it_cp : localMap)
				if (idxCPSeen.find(it_cp->getId()) == idxCPSeen.end())
					localMapUnseen.push_back(it_cp);
			std::vector< Point2Df > projected2DPts;
			projector->project(localMapUnseen, projected2DPts, pose);

			// find more inlier matches
			std::vector<SRef<DescriptorBuffer>> desAllLocalMapUnseen;
			for (auto &it_cp : localMapUnseen) {
				desAllLocalMapUnseen.push_back(it_cp->getDescriptor());
			}
			std::vector<DescriptorMatch> allMatches;
			matcher->matchInRegion(projected2DPts, desAllLocalMapUnseen, frame, allMatches, 0, maxMatchDistance * 1.5);
			// find visibility of new frame					
			for (auto &it_match : allMatches) {
				int idx_2d = it_match.getIndexInDescriptorB();
				int idx_3d = it_match.getIndexInDescriptorA();
				auto it2d = newMapVisibility.find(idx_2d);
				if (it2d == newMapVisibility.end()) {
					pts2d.push_back(Point2Df(keypoints[idx_2d].getX(), keypoints[idx_2d].getY()));
					newMapVisibility[idx_2d] = localMapUnseen[idx_3d]->getId();
				}
			}

			// Add visibilities to current frame
			frame->addVisibilities(newMapVisibility);
			LOG_INFO("Number of tracked points: {}", newMapVisibility.size());
			if (newMapVisibility.size() < minWeightNeighbor)
				break;

			// mapping
			SRef<Keyframe> keyframe;
			if (mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
				LOG_INFO("New keyframe id: {}", keyframe->getId());			
				// Local bundle adjustment
				std::vector<uint32_t> bestIdx;
				covisibilityGraph->getNeighbors(keyframe->getId(), minWeightNeighbor, bestIdx);
				bestIdx.push_back(keyframe->getId());
				LOG_INFO("Nb keyframe to local bundle: {}", bestIdx.size());
				double bundleReprojError = bundler->bundleAdjustment(camParams.intrinsic, camParams.distortion, bestIdx);
				// map pruning
				updateLocalMap(keyframe);
				mapper->pruning(localMap);
				// try to loop detection
				countNewKeyframes++;
				// loop closure
				if (countNewKeyframes >= 10) {
					SRef<Keyframe> detectedLoopKeyframe;
					Transform3Df sim3Transform;
					std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
					if (loopDetector->detect(keyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
						// detected loop keyframe
						LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
						LOG_INFO("Nb of duplicatedPointsIndices: {}", duplicatedPointsIndices.size());
						LOG_INFO("sim3Transform: \n{}", sim3Transform.matrix());
						// performs loop correction 						
						Transform3Df keyframeOldPose = keyframe->getPose();
						loopCorrector->correct(keyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);
						Transform3Df transform = keyframe->getPose() * keyframeOldPose.inverse();
						T_M_W = transform * T_M_W;
						// map pruning
						mapper->pruning();
						countNewKeyframes = 0;
					}
				}
			}			
			// update reference keyframe
			if (keyframe) {
				refKeyframe = keyframe;
				updateLocalMap(refKeyframe);
			}			
			// draw pose
			SRef<Image> displayImage = image->copy();
			overlay3D->draw(frame->getPose(), displayImage);
			overlay2DRed->drawCircles(pts2d_outliers, displayImage);
			overlay2DGreen->drawCircles(pts2d_inliers, displayImage);

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
			if (viewer3D->display(pointCloud, frame->getPose(), keyframePoses, framePoses, localMap) == FrameworkReturnCode::_STOP)
				break;
        }

		LOG_INFO("Nb of keyframes / cloud points: {} / {}", keyframesManager->getNbKeyframes(), pointCloudManager->getNbPoints());
		// display
		std::vector<Transform3Df> keyframePoses;
		std::vector<SRef<Keyframe>> allKeyframes;
		keyframesManager->getAllKeyframes(allKeyframes);
		for (auto const &it : allKeyframes)
			keyframePoses.push_back(it->getPose());
		std::vector<SRef<CloudPoint>> pointCloud;
		pointCloudManager->getAllPoints(pointCloud);
		while (true) {						 
			if (viewer3D->display(pointCloud, keyframePoses[0], keyframePoses, framePoses) == FrameworkReturnCode::_STOP)
				break;
		}

		// Save map
		mapper->saveToFile();
    }

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }
    return 0;
}
