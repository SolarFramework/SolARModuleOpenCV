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

#include "SolARDescriptorMatcherRegionOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include "opencv2/flann/kdtree_single_index.h"
#include "opencv2/flann.hpp"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorMatcherRegionOpencv)

namespace SolAR {
using namespace datastructure;
using namespace api::features;
namespace MODULES {
namespace OPENCV {

SolARDescriptorMatcherRegionOpencv::SolARDescriptorMatcherRegionOpencv():ConfigurableBase(xpcf::toUUID<SolARDescriptorMatcherRegionOpencv>())
{
    declareInterface<IDescriptorMatcherRegion>(this);
    declareProperty("distanceRatio", m_distanceRatio);
    declareProperty("radius", m_radius);
    declareProperty("matchingDistanceMax", m_matchingDistanceMax);
    LOG_DEBUG(" SolARDescriptorMatcherRegionOpencv constructor")
}

SolARDescriptorMatcherRegionOpencv::~SolARDescriptorMatcherRegionOpencv()
{
    LOG_DEBUG(" SolARDescriptorMatcherRegionOpencv destructor")
}

FrameworkReturnCode SolARDescriptorMatcherRegionOpencv::match(const std::vector<Point2Df>& points2D, const std::vector<SRef<DescriptorBuffer>>& descriptors, const SRef<Frame> frame, std::vector<DescriptorMatch>& matches, const float radius, const float matchingDistanceMax)
{
	matches.clear();
	// check if the descriptors type match
	if ((descriptors.size() == 0) || (frame->getDescriptors()->getNbDescriptors() == 0) ||
		(frame->getDescriptors()->getDescriptorType() != descriptors[0]->getDescriptorType())) {
		return FrameworkReturnCode::_ERROR_;
	}
	
	float radiusValue = radius > 0 ? radius : m_radius;
	float matchingDistanceMaxValue = matchingDistanceMax > 0 ? matchingDistanceMax : m_matchingDistanceMax;
	const SRef<DescriptorBuffer>& descriptorFrame = frame->getDescriptors();	

	uint32_t type_conversion = SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptorFrame->getDescriptorDataType());

	// convert frame descriptor to opencv's descriptor
	cv::Mat cvDescriptorFrame(descriptorFrame->getNbDescriptors(), descriptorFrame->getNbElements(), type_conversion);
	cvDescriptorFrame.data = (uchar*)descriptorFrame->data();

	std::vector<cv::Mat> cvDescriptors;
	for (unsigned k = 0; k < descriptors.size(); k++) {
		cv::Mat cvDescriptor(descriptors[k]->getNbDescriptors(), descriptors[k]->getNbElements(), type_conversion);
		cvDescriptor.data = (uchar*)(descriptors[k]->data());
		cvDescriptors.push_back(cvDescriptor);
	}

	// init kd tree to accelerate searching		
	const std::vector<Keypoint> &keypointsFrame = frame->getKeypoints();
	cv::Mat_<float> features(0, 2);
	for (const auto &kp : keypointsFrame) {
		cv::Mat row = (cv::Mat_<float>(1, 2) << kp.getX(), kp.getY());
		features.push_back(row);
	}
	cvflann::KDTreeSingleIndexParams indexParams;
	cv::flann::GenericIndex<cv::flann::L2<float>> kdtree(features, indexParams);
	// Match each descriptor to descriptors of frame in a corresponding region
	std::vector<bool> checkMatches(keypointsFrame.size(), true);
	int idx = 0;
	for (const auto &it : cvDescriptors) {
		std::vector<int> idxCandidates;
		std::vector<float> dists;
		std::vector<float> query = { points2D[idx].getX(), points2D[idx].getY() };
		std::vector<int> indicesMatrix(keypointsFrame.size());
		std::vector<float> distsMatrix(keypointsFrame.size());
		int nbFound = kdtree.radiusSearch(query, indicesMatrix, distsMatrix, radiusValue * radiusValue, cvflann::SearchParams());
		idxCandidates.assign(indicesMatrix.begin(), indicesMatrix.begin() + nbFound);
		dists.assign(distsMatrix.begin(), distsMatrix.begin() + nbFound);
		if (nbFound > 0) {
			float bestDist = std::numeric_limits<float>::max();
			float bestDist2 = std::numeric_limits<float>::max();
			int bestIdx = -1;
			for (const auto &it_des: idxCandidates) {
				float dist = cv::norm(it, cvDescriptorFrame.row(it_des), cv::NORM_L2);
				if (dist < bestDist)
				{
					bestDist2 = bestDist;
					bestDist = dist;
					bestIdx = it_des;
				}
				else if (dist < bestDist2)
				{
					bestDist2 = dist;
				}
			}
			if ((bestIdx != -1) && (bestDist < matchingDistanceMaxValue) && (bestDist < m_distanceRatio * bestDist2) && (checkMatches[bestIdx])) {
				matches.push_back(DescriptorMatch(idx, bestIdx, bestDist));
				checkMatches[bestIdx] = false;
			}
		}
		idx++;
	}		

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARDescriptorMatcherRegionOpencv::match(const SRef<datastructure::Frame> currentFrame, const SRef<datastructure::Frame> lastFrame, std::vector<datastructure::DescriptorMatch>& matches, const float radius, const float matchingDistanceMax)
{		
	matches.clear();
	// check if the descriptors type match
	if ((currentFrame->getDescriptors()->getDescriptorType() != lastFrame->getDescriptors()->getDescriptorType()) ||
		(currentFrame->getDescriptors()->getNbDescriptors() == 0) || (lastFrame->getDescriptors()->getNbDescriptors() == 0)) {
		return FrameworkReturnCode::_ERROR_;
	}

	float radiusValue = radius > 0 ? radius : m_radius;
	float matchingDistanceMaxValue = matchingDistanceMax > 0 ? matchingDistanceMax : m_matchingDistanceMax;
	SRef<DescriptorBuffer> desCurrentFrame = currentFrame->getDescriptors();
	SRef<DescriptorBuffer> desLastFrame = lastFrame->getDescriptors();

	uint32_t type_conversion = SolAROpenCVHelper::deduceOpenDescriptorCVType(desCurrentFrame->getDescriptorDataType());

	// convert frame descriptor to opencv's descriptor
	cv::Mat cvDesCurrentFrame(desCurrentFrame->getNbDescriptors(), desCurrentFrame->getNbElements(), type_conversion);
	cvDesCurrentFrame.data = (uchar*)desCurrentFrame->data();

	cv::Mat cvDesLastFrame(desLastFrame->getNbDescriptors(), desLastFrame->getNbElements(), type_conversion);
	cvDesLastFrame.data = (uchar*)desLastFrame->data();

	// init kd tree to accelerate searching		
	const std::vector<Keypoint> &keypointsLastFrame = lastFrame->getKeypoints();
	cv::Mat_<float> features(0, 2);
	for (const auto &kp : keypointsLastFrame) {
		cv::Mat row = (cv::Mat_<float>(1, 2) << kp.getX(), kp.getY());
		features.push_back(row);
	}
	cvflann::KDTreeSingleIndexParams indexParams;
	cv::flann::GenericIndex<cv::flann::L2<float>> kdtree(features, indexParams);

	// Match each descriptor of the current frame to descriptors of the last frame in a corresponding region
	std::vector<bool> checkMatches(keypointsLastFrame.size(), true);
	const std::vector<Keypoint> &keypointsCurrentFrame = currentFrame->getKeypoints();
	for (int i = 0; i < keypointsCurrentFrame.size(); ++i) {
		std::vector<int> idxCandidates;
		std::vector<float> dists;
		std::vector<float> query = { keypointsCurrentFrame[i].getX(), keypointsCurrentFrame[i].getY() };
		std::vector<int> indicesMatrix(keypointsLastFrame.size());
		std::vector<float> distsMatrix(keypointsLastFrame.size());
		int nbFound = kdtree.radiusSearch(query, indicesMatrix, distsMatrix, radiusValue * radiusValue, cvflann::SearchParams());
		idxCandidates.assign(indicesMatrix.begin(), indicesMatrix.begin() + nbFound);
		dists.assign(distsMatrix.begin(), distsMatrix.begin() + nbFound);
		cv::Mat queryDes = cvDesCurrentFrame.row(i);
		if (nbFound > 0) {
			float bestDist = std::numeric_limits<float>::max();
			float bestDist2 = std::numeric_limits<float>::max();
			int bestIdx = -1;
			for (const auto &idx_des: idxCandidates) {
				float dist = cv::norm(queryDes, cvDesLastFrame.row(idx_des), cv::NORM_L2);
				if (dist < bestDist)
				{
					bestDist2 = bestDist;
					bestDist = dist;
					bestIdx = idx_des;
				}
				else if (dist < bestDist2)
				{
					bestDist2 = dist;
				}
			}
			if ((bestIdx != -1) && (bestDist < matchingDistanceMaxValue) && (bestDist < m_distanceRatio * bestDist2) && (checkMatches[bestIdx])) {
				matches.push_back(DescriptorMatch(i, bestIdx, bestDist));
				checkMatches[bestIdx] = false;
			}
		}
	}
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}  // end of namespace SolAR
