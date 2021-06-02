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

#include "SolARMapFusionOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include "opencv2/flann/kdtree_single_index.h"
#include "opencv2/flann.hpp"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARMapFusionOpencv)

namespace SolAR {
using namespace datastructure;
using namespace api::solver::map;
namespace MODULES {
namespace OPENCV {

SolARMapFusionOpencv::SolARMapFusionOpencv() :ConfigurableBase(xpcf::toUUID<SolARMapFusionOpencv>())
{
	declareInterface<IMapFusion>(this);
	declareInjectable<api::geom::I3DTransform>(m_transform3D);
	declareInjectable<api::features::IDescriptorMatcher>(m_matcher);
	declareInjectable<api::solver::pose::I3DTransformSACFinderFrom3D3D>(m_estimator3D);
	declareProperty("radius", m_radius);
	LOG_DEBUG(" SolARMapFusionOpencv constructor")
}

SolARMapFusionOpencv::~SolARMapFusionOpencv()
{
	LOG_DEBUG(" SolARMapFusionOpencv destructor")
}

FrameworkReturnCode SolARMapFusionOpencv::merge(SRef<datastructure::Map> map, SRef<datastructure::Map> globalMap, Transform3Df & transform, uint32_t & nbMatches, float & error)
{
	/// Transform local map to global map
	m_transform3D->transformInPlace(transform, map);
	
	// get point cloud
	std::vector<SRef<CloudPoint>> cloudPoints, globalCloudPoints;
	const SRef<PointCloud>& pointCloud = map->getConstPointCloud();
	pointCloud->getAllPoints(cloudPoints);
	const SRef<PointCloud>& globalPointCloud = globalMap->getConstPointCloud();
	globalPointCloud->getAllPoints(globalCloudPoints);

	/// Find 3D-3D correspondences using kd tree and feature
	// init kd tree of global point cloud
	cv::Mat_<float> features(0, 3);
	for (const auto &cp : globalCloudPoints) {
		cv::Mat row = (cv::Mat_<float>(1, 3) << cp->getX(), cp->getY(), cp->getZ());
		features.push_back(row);
	}
	cvflann::KDTreeSingleIndexParams indexParams;
	cv::flann::GenericIndex<cv::flann::L2<float>> kdtree(features, indexParams);
	// find correspondences for each point
	std::vector < std::pair<SRef<CloudPoint>, SRef<CloudPoint>>> duplicatedCPs; // first is local CP, second is global CP
	std::vector<bool> checkMatches(globalCloudPoints.size(), true);
	for (auto &cp : cloudPoints) {
		// find point by 3D distance
		std::vector<int> idxCandidates;
		std::vector<float> dists;
		std::vector<float> pt3D = { cp->getX(), cp->getY(), cp->getZ() };
		std::vector<int> indicesMatrix(globalCloudPoints.size());
		std::vector<float> distsMatrix(globalCloudPoints.size());
		int nbFound = kdtree.radiusSearch(pt3D, indicesMatrix, distsMatrix, m_radius * m_radius, cvflann::SearchParams());
		idxCandidates.assign(indicesMatrix.begin(), indicesMatrix.begin() + nbFound);
		dists.assign(distsMatrix.begin(), distsMatrix.begin() + nbFound);
		std::vector<SRef<DescriptorBuffer>> desCandidates;
		std::vector<int> idxBestCandidates;
		for (const auto &idx : idxCandidates) {
			if (checkMatches[idx]) {
				desCandidates.push_back(globalCloudPoints[idx]->getDescriptor());
				idxBestCandidates.push_back(idx);
			}
		}
		// filter by descriptor distance
		if (desCandidates.size() > 0) {
			std::vector<DescriptorMatch> matches;
			m_matcher->match(cp->getDescriptor(), desCandidates, matches);
			if (matches.size() != 0) {
				int idxMatch = matches[0].getIndexInDescriptorB();
				checkMatches[idxBestCandidates[idxMatch]] = false;
				duplicatedCPs.push_back(std::make_pair(cp, globalCloudPoints[idxBestCandidates[idxMatch]]));
			}
		}
	}	

	/// Estimate transform2
	std::vector<Point3Df> firstPts3D, secondPts3D;
	for (const auto &it : duplicatedCPs) {
		firstPts3D.push_back(Point3Df(it.first->getX(), it.first->getY(), it.first->getZ()));
		secondPts3D.push_back(Point3Df(it.second->getX(), it.second->getY(), it.second->getZ()));
	}
	Transform3Df transform2;
	std::vector<int> inliers;
	m_estimator3D->estimate(firstPts3D, secondPts3D, transform2, inliers);	
	if (inliers.size() == 0)
		return FrameworkReturnCode::_ERROR_;

	/// Apply transform2 and refine transform
	m_transform3D->transformInPlace(transform2, map);
	transform = transform2 * transform;

	/// get best matches
    nbMatches = static_cast<uint32_t>(inliers.size());
	error = 0.f;
	std::vector < std::pair<uint32_t, uint32_t>> duplicatedIndiceCPsFiltered; // first is indice of local CP, second is indice of global CP
	for (const auto &it : inliers) {
		duplicatedIndiceCPsFiltered.push_back(std::make_pair(duplicatedCPs[it].first->getId(), duplicatedCPs[it].second->getId()));
		Point3Df pt1(duplicatedCPs[it].first->getX(), duplicatedCPs[it].first->getY(), duplicatedCPs[it].first->getZ());
		Point3Df pt2(duplicatedCPs[it].second->getX(), duplicatedCPs[it].second->getY(), duplicatedCPs[it].second->getZ());
		error += (pt1 - pt2).norm();
	}
	error /= nbMatches;

	/// fuse local map into global map
	fuseMap(duplicatedIndiceCPsFiltered, map, globalMap);

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapFusionOpencv::merge(SRef<datastructure::Map> map, SRef<datastructure::Map> globalMap, Transform3Df & transform, const std::vector<std::pair<uint32_t, uint32_t>>& cpOverlapIndices, bool isRefineTransform)
{
	uint32_t nbMatches;
	float error;
	if (isRefineTransform)
		return this->merge(map, globalMap, transform, nbMatches, error);
	// transform map
	m_transform3D->transformInPlace(transform, map);
	// fuse local map into global map
	fuseMap(cpOverlapIndices, map, globalMap);
	return FrameworkReturnCode::_SUCCESS;
}

void SolARMapFusionOpencv::fuseMap(const std::vector<std::pair<uint32_t, uint32_t>>& cpOverlapIndices, SRef<datastructure::Map> map, SRef<datastructure::Map> globalMap)
{
	// get point cloud
	std::vector<SRef<CloudPoint>> cloudPoints, globalCloudPoints;
	const SRef<PointCloud>& pointCloud = map->getConstPointCloud();
	pointCloud->getAllPoints(cloudPoints);
	SRef<PointCloud> globalPointCloud;
	globalMap->getPointCloud(globalPointCloud);
	globalPointCloud->getAllPoints(globalCloudPoints);

	// get keyframes
	std::vector<SRef<Keyframe>> keyframes, globalKeyframes;
	const SRef<KeyframeCollection>& keyframeCollection = map->getConstKeyframeCollection();
	keyframeCollection->getAllKeyframes(keyframes);
	SRef<KeyframeCollection> globalKeyframeCollection;
	globalMap->getKeyframeCollection(globalKeyframeCollection);
	globalKeyframeCollection->getAllKeyframes(globalKeyframes);

	// get covisibility graph
	const SRef<CovisibilityGraph>& covisibilityGraph = map->getConstCovisibilityGraph();
	SRef<CovisibilityGraph> globalCovisibilityGraph;
	globalMap->getCovisibilityGraph(globalCovisibilityGraph);

	// get keyframe retrieval	
	const SRef<KeyframeRetrieval>& keyframeRetrieval = map->getConstKeyframeRetrieval();
	SRef<KeyframeRetrieval> globalKeyframeRetrieval;
	globalMap->getKeyframeRetrieval(globalKeyframeRetrieval);

	// get duplicated cloud points
	std::vector < std::pair<SRef<CloudPoint>, SRef<CloudPoint>>> duplicatedCPsFiltered; // first is local CP, second is global CP
	for (const auto &it : cpOverlapIndices) {
		SRef<CloudPoint> localCP, globalCP;
		pointCloud->getPoint(it.first, localCP);
		globalPointCloud->getPoint(it.second, globalCP);
		duplicatedCPsFiltered.push_back(std::make_pair(localCP, globalCP));
	}

	// add point cloud of local map to global map
	std::map<uint32_t, uint32_t> idxCPOldNew;
	for (const auto &cp : cloudPoints) {
		uint32_t idxOld = cp->getId();
		globalPointCloud->addPoint(cp);
		idxCPOldNew[idxOld] = cp->getId();
	}

	// add keyframes of local map to global map
	std::map<uint32_t, uint32_t> idxKfOldNew;
	for (const auto &kf : keyframes) {
		// unscale keyframe pose
		Transform3Df kfPose = kf->getPose();
		Eigen::Matrix3f scale;
		Eigen::Matrix3f rot;
		kfPose.computeScalingRotation(&scale, &rot);
		kfPose.linear() = rot;
		kf->setPose(kfPose);

		uint32_t idxOld = kf->getId();
		globalKeyframeCollection->addKeyframe(kf);
		idxKfOldNew[idxOld] = kf->getId();
	}

	// update visibilities of point cloud
	for (const auto &cp : cloudPoints) {
		std::map<uint32_t, uint32_t> visibilities = cp->getVisibility();
		for (const auto &vi : visibilities) {
			cp->removeVisibility(vi.first);
			cp->addVisibility(idxKfOldNew[vi.first], vi.second);
		}
	}

	// update visibilities of keyframe
	for (const auto &kf : keyframes) {
		std::map<uint32_t, uint32_t> visibilities = kf->getVisibility();
		for (const auto &vi : visibilities) {
			kf->removeVisibility(vi.first, vi.second);
			kf->addVisibility(vi.first, idxCPOldNew[vi.second]);
		}
	}

	// add keyframe retriever of local map to global map
	for (const auto &idKf : idxKfOldNew) {
        datastructure::BoWFeature bowDesc;
        datastructure::BoWLevelFeature bowLevelDesc;
        keyframeRetrieval->getBoWFeature(idKf.first, bowDesc);
        keyframeRetrieval->getBoWLevelFeature(idKf.first, bowLevelDesc);
        globalKeyframeRetrieval->addDescriptor(idKf.second, bowDesc, bowLevelDesc);
	}

	// add covisibility graph of local map to global map
	for (auto it1 = idxKfOldNew.begin(); std::next(it1) != idxKfOldNew.end(); ++it1)
		for (auto it2 = std::next(it1); it2 != idxKfOldNew.end(); ++it2) {
			float weight;
			if (covisibilityGraph->getEdge(it1->first, it2->first, weight) == FrameworkReturnCode::_SUCCESS)
				globalCovisibilityGraph->increaseEdge(it1->second, it2->second, weight);
		}

	// Fuse duplicated cloud points
	for (const auto &dup : duplicatedCPsFiltered) {
		SRef<CloudPoint> cp1 = dup.first;
		SRef<CloudPoint> cp2 = dup.second;
		const std::map<uint32_t, uint32_t> &visibilities1 = cp1->getVisibility();
		const std::map<uint32_t, uint32_t> &visibilities2 = cp2->getVisibility();
		for (const auto &vi1 : visibilities1) {
			uint32_t id_kf1 = vi1.first;
			uint32_t id_kp1 = vi1.second;
			SRef<Keyframe> kf1;
			// update visibility of keyframes seen cp1
			globalKeyframeCollection->getKeyframe(id_kf1, kf1);
			kf1->addVisibility(id_kp1, cp2->getId());
			// move visibility of cp1 to cp2
			cp2->addVisibility(id_kf1, id_kp1);
			// update covisibility graph
			for (const auto &vi2 : visibilities2) {
				uint32_t id_kf2 = vi2.first;
				globalCovisibilityGraph->increaseEdge(id_kf1, id_kf2, 1.0);
			}
		}
		// suppress cp1
		globalPointCloud->suppressPoint(cp1->getId());
	}
}

}
}
}  // end of namespace SolAR
