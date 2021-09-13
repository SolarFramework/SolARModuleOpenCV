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

SolARDescriptorMatcherRegionOpencv::SolARDescriptorMatcherRegionOpencv(): base::features::ADescriptorMatcherRegion(xpcf::toMap<SolARDescriptorMatcherRegionOpencv>())
{
    declareProperty("distanceRatio", m_distanceRatio);
    declareProperty("radius", m_radius);
    declareProperty("matchingDistanceMax", m_matchingDistanceMax);
    LOG_DEBUG(" SolARDescriptorMatcherRegionOpencv constructor")
}

SolARDescriptorMatcherRegionOpencv::~SolARDescriptorMatcherRegionOpencv()
{
    LOG_DEBUG(" SolARDescriptorMatcherRegionOpencv destructor")
}

FrameworkReturnCode SolARDescriptorMatcherRegionOpencv::match(const SRef<SolAR::datastructure::DescriptorBuffer> descriptors1, 
	const SRef<SolAR::datastructure::DescriptorBuffer> descriptors2, 
	const std::vector<SolAR::datastructure::Point2Df>& points2D1, 
	const std::vector<SolAR::datastructure::Point2Df>& points2D2, 
	std::vector<SolAR::datastructure::DescriptorMatch>& matches, 
	const float radius, 
	const float matchingDistanceMax)
{
	// check conditions
	if ((descriptors1->getDescriptorType() != descriptors2->getDescriptorType()) ||
		(descriptors1->getNbDescriptors() == 0) || (descriptors2->getNbDescriptors() == 0)) {
		return FrameworkReturnCode::_ERROR_;
	}
	float radiusValue = radius > 0 ? radius : m_radius;
	float matchingDistanceMaxValue = matchingDistanceMax > 0 ? matchingDistanceMax : m_matchingDistanceMax;
	// convert frame descriptor to opencv's descriptor
	uint32_t type_conversion = SolAR::MODULES::OPENCV::SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptors1->getDescriptorDataType());
	cv::Mat cvDescriptor1(descriptors1->getNbDescriptors(), descriptors1->getNbElements(), type_conversion);
	cvDescriptor1.data = (uchar*)descriptors1->data();
	cv::Mat cvDescriptor2(descriptors2->getNbDescriptors(), descriptors2->getNbElements(), type_conversion);
	cvDescriptor2.data = (uchar*)descriptors2->data();
	// init kd tree to accelerate searching		
	cv::Mat_<float> features(0, 2);
	for (const auto &pt : points2D2) {
		cv::Mat row = (cv::Mat_<float>(1, 2) << pt.getX(), pt.getY());
		features.push_back(row);
	}
	cvflann::KDTreeSingleIndexParams indexParams;
	cv::flann::GenericIndex<cv::flann::L2<float>> kdtree(features, indexParams);
	// Match each descriptor to descriptors of frame in a corresponding region
	std::vector<bool> checkMatches(points2D2.size(), true);
	int idx = 0;
	for (int idx = 0; idx < points2D1.size(); ++idx) {
		std::vector<int> idxCandidates;
		std::vector<float> dists;
		std::vector<float> query = { points2D1[idx].getX(), points2D1[idx].getY() };
		std::vector<int> indicesMatrix(points2D2.size());
		std::vector<float> distsMatrix(points2D2.size());
		int nbFound = kdtree.radiusSearch(query, indicesMatrix, distsMatrix, radiusValue * radiusValue, cvflann::SearchParams());
		idxCandidates.assign(indicesMatrix.begin(), indicesMatrix.begin() + nbFound);
		dists.assign(distsMatrix.begin(), distsMatrix.begin() + nbFound);
		if (nbFound > 0) {
			float bestDist = std::numeric_limits<float>::max();
			float bestDist2 = std::numeric_limits<float>::max();
			int bestIdx = -1;
			for (const auto &it_des : idxCandidates) {
				float dist = cv::norm(cvDescriptor1.row(idx), cvDescriptor2.row(it_des), cv::NORM_L2);
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

}
}
}  // end of namespace SolAR
