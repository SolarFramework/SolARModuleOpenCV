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

#include "SolARDescriptorMatcherKNNOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include "opencv2/flann/kdtree_single_index.h"
#include "opencv2/flann.hpp"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorMatcherKNNOpencv)

namespace SolAR {
using namespace datastructure;
using namespace api::features;
namespace MODULES {
namespace OPENCV {

SolARDescriptorMatcherKNNOpencv::SolARDescriptorMatcherKNNOpencv():ConfigurableBase(xpcf::toUUID<SolARDescriptorMatcherKNNOpencv>())
{
    declareInterface<IDescriptorMatcher>(this);
    declareProperty("distanceRatio", m_distanceRatio);
    declareProperty("radius", m_radius);
    declareProperty("matchingDistanceMax", m_matchingDistanceMax);
    LOG_DEBUG(" SolARDescriptorMatcherKNNOpencv constructor")
}

SolARDescriptorMatcherKNNOpencv::~SolARDescriptorMatcherKNNOpencv()
{
    LOG_DEBUG(" SolARDescriptorMatcherKNNOpencv destructor")
}

cv::flann::GenericIndex<cv::flann::L2<float>> initKDTree(const std::vector<Keypoint> &keypoints)
{
	cv::Mat_<float> features(0, 2);
	for (const auto &kp : keypoints) {
		cv::Mat row = (cv::Mat_<float>(1, 2) << kp.getX(), kp.getY());
		features.push_back(row);
	}
	cvflann::KDTreeSingleIndexParams indexParams;
	cv::flann::GenericIndex<cv::flann::L2<float>> kdtree(features, indexParams);
	return kdtree;
}

int radiusSearch(cv::flann::GenericIndex<cv::flann::L2<float>>& kdtree, const Point2Df& query, const float & radius, std::vector<int> & indices, std::vector<float> & dists, const int & maxResults = 500)
{
	std::vector<float> pt2D = { query.getX(), query.getY() };
	std::vector<int> tmpIndices(maxResults);
	std::vector<float> tmpDists(maxResults);
	LOG_DEBUG("OK__4: {}", tmpIndices.size());
	int nbFound = kdtree.radiusSearch(pt2D, tmpIndices, tmpDists, radius * radius, cvflann::SearchParams());
	LOG_DEBUG("OK__4: {}", nbFound);
	indices.assign(tmpIndices.begin(), tmpIndices.begin() + nbFound);
	dists.assign(tmpDists.begin(), tmpDists.begin() + nbFound);
	return nbFound;
}

IDescriptorMatcher::RetCode SolARDescriptorMatcherKNNOpencv::match(
            SRef<DescriptorBuffer> desc1,SRef<DescriptorBuffer> desc2, std::vector<DescriptorMatch>& matches){

    matches.clear();

    // check if the descriptors type match
    if(desc1->getDescriptorType() != desc2->getDescriptorType()){
        return IDescriptorMatcher::RetCode::DESCRIPTORS_DONT_MATCH;
    }

    if (desc1->getNbDescriptors() == 0 || desc2->getNbDescriptors() == 0){
        return IDescriptorMatcher::RetCode::DESCRIPTOR_EMPTY;
    }

    if (desc1->getNbDescriptors()<2 || desc2->getNbDescriptors()<2) {
        matches.clear();
        return IDescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK;  // not enough descriptors to use opencv::knnMatch
    }

    std::vector<std::vector<cv::DMatch>> initial_matches;
    std::vector<cv::DMatch> good_matches;

    //since it is an openCV implementation we need to convert back the descriptors from SolAR to Opencv
    uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(desc1->getDescriptorDataType());

    cv::Mat cvDescriptor1(desc1->getNbDescriptors(), desc1->getNbElements(), type_conversion);
    cvDescriptor1.data=(uchar*)desc1->data();

    cv::Mat cvDescriptor2(desc2->getNbDescriptors(), desc1->getNbElements(), type_conversion);
    cvDescriptor2.data=(uchar*)desc2->data();

    if (desc1->getDescriptorDataType() != DescriptorDataType::TYPE_32F)
        cvDescriptor1.convertTo(cvDescriptor1, CV_32F);
    if (desc2->getDescriptorDataType() != DescriptorDataType::TYPE_32F)
        cvDescriptor2.convertTo(cvDescriptor2, CV_32F);

    std::vector< std::vector<cv::DMatch> > nn_matches;
    m_matcher.knnMatch(cvDescriptor1, cvDescriptor2, nn_matches,2);
	std::map<uint32_t, std::map<uint32_t, float>> matches21;
    for(unsigned i = 0; i < nn_matches.size(); i++) {
        if(nn_matches[i][0].distance < m_distanceRatio * nn_matches[i][1].distance) {
			matches21[nn_matches[i][0].trainIdx][nn_matches[i][0].queryIdx] = nn_matches[i][0].distance;
        }
    }

	// get best matches to descriptors 2
	for (auto it_des2 : matches21) {
		uint32_t idxDes2 = it_des2.first;
		std::map<uint32_t, float> infoMatch = it_des2.second;
		uint32_t bestIdxDes1;
		float bestDistance = FLT_MAX;
		for (auto it_des1: infoMatch)
			if (it_des1.second < bestDistance) {
				bestDistance = it_des1.second;
				bestIdxDes1 = it_des1.first;
			}
		matches.push_back(DescriptorMatch(bestIdxDes1, idxDes2, bestDistance));
	}

    return IDescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK;

}

IDescriptorMatcher::RetCode SolARDescriptorMatcherKNNOpencv::match(
            const SRef<DescriptorBuffer> descriptors1,
            const std::vector<SRef<DescriptorBuffer>>& descriptors2,
            std::vector<DescriptorMatch>& matches
            )
{

    matches.clear();

    if (descriptors1->getNbDescriptors() ==0 || descriptors2.size()== 0)
        return IDescriptorMatcher::RetCode::DESCRIPTOR_EMPTY;

    uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptors1->getDescriptorDataType());

    cv::Mat cvDescriptors1(descriptors1->getNbDescriptors(), descriptors1->getNbElements(), type_conversion);
    cvDescriptors1.data=(uchar*)descriptors1->data();

    if (descriptors1->getDescriptorDataType() != DescriptorDataType::TYPE_32F)
        cvDescriptors1.convertTo(cvDescriptors1, CV_32F);

    std::vector<cv::Mat> cvDescriptors;
    for(unsigned k=0;k<descriptors2.size();k++){

        uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptors2[k]->getDescriptorDataType());

        cv::Mat cvDescriptor(descriptors2[k]->getNbDescriptors(), descriptors2[k]->getNbElements(), type_conversion);
        cvDescriptor.data=(uchar*)(descriptors2[k]->data());

        if (descriptors2[k]->getDescriptorDataType() != DescriptorDataType::TYPE_32F)
            cvDescriptor.convertTo(cvDescriptor, CV_32F);

        cvDescriptors.push_back(cvDescriptor);
    }


    cv::Mat cvDescriptors2;
    cv::vconcat(cvDescriptors,cvDescriptors2);


    int nbOfMatches=2;

    if(cvDescriptors2.rows<nbOfMatches)
        return IDescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK;

    std::vector< std::vector<cv::DMatch> > nn_matches;
    m_matcher.knnMatch(cvDescriptors1, cvDescriptors2, nn_matches,nbOfMatches);
	std::map<uint32_t, std::map<uint32_t, float>> matches21;
    for(unsigned i = 0; i < nn_matches.size(); i++) {
        if(nn_matches[i][0].distance < m_distanceRatio * nn_matches[i][1].distance) {
			matches21[nn_matches[i][0].trainIdx][nn_matches[i][0].queryIdx] = nn_matches[i][0].distance;
        }
    }
	// get best matches to descriptors 2
	for (auto it_des2 : matches21) {
		uint32_t idxDes2 = it_des2.first;
		std::map<uint32_t, float> infoMatch = it_des2.second;
		uint32_t bestIdxDes1;
		float bestDistance = FLT_MAX;
		for (auto it_des1 : infoMatch)
			if (it_des1.second < bestDistance) {
				bestDistance = it_des1.second;
				bestIdxDes1 = it_des1.first;
			}
		matches.push_back(DescriptorMatch(bestIdxDes1, idxDes2, bestDistance));
	}
    return IDescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK;

}

IDescriptorMatcher::RetCode SolARDescriptorMatcherKNNOpencv::matchInRegion(const std::vector<Point2Df>& points2D, const std::vector<SRef<DescriptorBuffer>>& descriptors, const SRef<Frame> frame, std::vector<DescriptorMatch>& matches, const float radius, const float matchingDistanceMax)
{
	matches.clear();
	float radiusValue = radius > 0 ? radius : m_radius;
	float matchingDistanceMaxValue = matchingDistanceMax > 0 ? matchingDistanceMax : m_matchingDistanceMax;
	const SRef<DescriptorBuffer>& descriptorFrame = frame->getDescriptors();

	if (descriptors.size() == 0 || descriptorFrame->getNbDescriptors() == 0)
		return IDescriptorMatcher::RetCode::DESCRIPTOR_EMPTY;

	// check if the descriptors type match
	if (descriptorFrame->getDescriptorType() != descriptors[0]->getDescriptorType()) {
		return IDescriptorMatcher::RetCode::DESCRIPTORS_DONT_MATCH;
	}

	if (points2D.size() != descriptors.size()) {			
		return IDescriptorMatcher::RetCode::DESCRIPTOR_TYPE_UNDEFINED;
	}		

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

	return IDescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK;
}

IDescriptorMatcher::RetCode SolARDescriptorMatcherKNNOpencv::matchInRegion(const SRef<datastructure::Frame> currentFrame, const SRef<datastructure::Frame> lastFrame, std::vector<datastructure::DescriptorMatch>& matches, const float radius, const float matchingDistanceMax)
{		
	matches.clear();
	float radiusValue = radius > 0 ? radius : m_radius;
	float matchingDistanceMaxValue = matchingDistanceMax > 0 ? matchingDistanceMax : m_matchingDistanceMax;
	SRef<DescriptorBuffer> desCurrentFrame = currentFrame->getDescriptors();
	SRef<DescriptorBuffer> desLastFrame = lastFrame->getDescriptors();
	if ((desCurrentFrame->getNbDescriptors() == 0) || (desLastFrame->getNbDescriptors() == 0))
		return IDescriptorMatcher::RetCode::DESCRIPTOR_EMPTY;

	// check if the descriptors type match
	if (desCurrentFrame->getDescriptorType() != desLastFrame->getDescriptorType()) {
		return IDescriptorMatcher::RetCode::DESCRIPTORS_DONT_MATCH;
	}

	uint32_t type_conversion = SolAROpenCVHelper::deduceOpenDescriptorCVType(desCurrentFrame->getDescriptorDataType());

	// convert frame descriptor to opencv's descriptor
	cv::Mat cvDesCurrentFrame(desCurrentFrame->getNbDescriptors(), desCurrentFrame->getNbElements(), type_conversion);
	cvDesCurrentFrame.data = (uchar*)desCurrentFrame->data();

	cv::Mat cvDesLastFrame(desLastFrame->getNbDescriptors(), desLastFrame->getNbElements(), type_conversion);
	cvDesLastFrame.data = (uchar*)desLastFrame->data();

	// init kd tree to accelerate searching		
	const std::vector<Keypoint> &keypointsLastFrame = lastFrame->getKeypoints();
	auto kdtree = initKDTree(keypointsLastFrame);

	// Match each descriptor of the current frame to descriptors of the last frame in a corresponding region
	std::vector<bool> checkMatches(keypointsLastFrame.size(), true);
	const std::vector<Keypoint> &keypointsCurrentFrame = currentFrame->getKeypoints();
	for (int i = 0; i < keypointsCurrentFrame.size(); ++i) {
		std::vector<int> idxCandidates;
		std::vector<float> dists;
		Point2Df queryPt2D(keypointsCurrentFrame[i].getX(), keypointsCurrentFrame[i].getY());
		cv::Mat queryDes = cvDesCurrentFrame.row(i);
		int nbFound = radiusSearch(kdtree, queryPt2D, radiusValue, idxCandidates, dists, keypointsLastFrame.size());
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
	return IDescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK;
}

}
}
}  // end of namespace SolAR
