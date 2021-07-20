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

#include "SolARStereoDescriptorMatcherOpencv.h"
#include "core/Log.h"
#include "SolAROpenCVHelper.h"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARStereoDescriptorMatcherOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARStereoDescriptorMatcherOpencv::SolARStereoDescriptorMatcherOpencv() : base::features::ADescriptorMatcherStereo(xpcf::toMap<SolARStereoDescriptorMatcherOpencv>())
{
    declareInterface<api::features::IDescriptorMatcherStereo>(this);
	declareProperty("ratioRadius", m_ratioRadius);
	declareProperty("matchingDistanceMax", m_matchingDistanceMax);
	LOG_DEBUG("SolARStereoDescriptorMatcherOpencv constructor");
}

SolARStereoDescriptorMatcherOpencv::~SolARStereoDescriptorMatcherOpencv()
{
	LOG_DEBUG("SolARStereoDescriptorMatcherOpencv destructor");
}

FrameworkReturnCode SolARStereoDescriptorMatcherOpencv::match(const SRef<SolAR::datastructure::DescriptorBuffer>& descriptors1, const SRef<SolAR::datastructure::DescriptorBuffer>& descriptors2, const std::vector<SolAR::datastructure::Keypoint>& keypoints1, const std::vector<SolAR::datastructure::Keypoint>& keypoints2, SolAR::datastructure::StereoType type, std::vector<SolAR::datastructure::DescriptorMatch>& matches)
{
	// convert descriptors to Mat
	uint32_t type_conversion = SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptors1->getDescriptorDataType());
	cv::Mat cvDes1(descriptors1->getNbDescriptors(), descriptors1->getNbElements(), type_conversion);
	cvDes1.data = (uchar*)descriptors1->data();
	cv::Mat cvDes2(descriptors2->getNbDescriptors(), descriptors2->getNbElements(), type_conversion);
	cvDes2.data = (uchar*)descriptors2->data();
	// match
	if (type == StereoType::Horizontal)
		return horizontalMatch(cvDes1, cvDes2, keypoints1, keypoints2, matches);
	else
		return verticalMatch(cvDes1, cvDes2, keypoints1, keypoints2, matches);
}

bool SolARStereoDescriptorMatcherOpencv::findBestMatch(const cv::Mat & query, const cv::Mat & descs, const std::vector<int>& candidatesId, int& bestIdx, float &minDist)
{
	bestIdx = -1;
	if (candidatesId.size() == 0)
		return false;
	minDist = m_matchingDistanceMax;
	for (int i = 0; i < candidatesId.size(); ++i) {
		float dist = cv::norm(query, descs.row(candidatesId[i]), cv::NORM_L2);
		if (dist < minDist) {
			minDist = dist;
			bestIdx = candidatesId[i];
		}
	}
	if (bestIdx == -1)
		return false;
	else
		return true;
}

FrameworkReturnCode SolARStereoDescriptorMatcherOpencv::horizontalMatch(const cv::Mat& descriptors1, const cv::Mat& descriptors2, const std::vector<SolAR::datastructure::Keypoint>& keypoints1, const std::vector<SolAR::datastructure::Keypoint>& keypoints2, std::vector<SolAR::datastructure::DescriptorMatch>& matches)
{
	int maxValue = 0;
	for (const auto& kp : keypoints1)
		maxValue = std::max(maxValue, (int)kp.getY());
	// radius pixels
	int r = maxValue * m_ratioRadius;
	// build table of indices to accelerate matching
	std::map<int, std::vector<int>> tabIndices;
	for (int i = 0; i < keypoints2.size(); ++i) {
		const Keypoint& kp = keypoints2[i];
		int min = (int)kp.getY() - r;
		int max = (int)kp.getY() + r;
		for (int j = min; j <= max; ++j)
			tabIndices[j].push_back(i);
	}

	// match each descriptor of the first image to descriptors of the second image
	std::vector<bool> checkMatches(keypoints2.size(), true);
	for (int i = 0; i < keypoints1.size(); ++i) {
		const Keypoint& kp1 = keypoints1[i];
		std::map<int, std::vector<int>>::iterator it_candidates = tabIndices.find((int)kp1.getY());
		if (it_candidates == tabIndices.end())
			continue;
		const std::vector<int>& candidates = it_candidates->second;
		int bestIdx;
		float minDist;
		if (findBestMatch(descriptors1.row(i), descriptors2, candidates, bestIdx, minDist) && checkMatches[bestIdx]) {
			checkMatches[bestIdx] = false;
			matches.push_back(DescriptorMatch(i, bestIdx, minDist));
		}
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARStereoDescriptorMatcherOpencv::verticalMatch(const cv::Mat& descriptors1, const cv::Mat& descriptors2, const std::vector<SolAR::datastructure::Keypoint>& keypoints1, const std::vector<SolAR::datastructure::Keypoint>& keypoints2, std::vector<SolAR::datastructure::DescriptorMatch>& matches)
{
	int maxValue = 0;
	for (const auto& kp : keypoints1)
		maxValue = std::max(maxValue, (int)kp.getX());
	// radius pixels
	int r = std::max((int)(maxValue * m_ratioRadius), 3);
	// build table of indices to accelerate matching
	std::map<int, std::vector<int>> tabIndices;
	for (int i = 0; i < keypoints2.size(); ++i) {
		const Keypoint& kp = keypoints2[i];
		int min = (int)kp.getX() - r;
		int max = (int)kp.getX() + r;
		for (int j = min; j <= max; ++j)
			tabIndices[j].push_back(i);
	}
	
	// match each descriptor of the first image to descriptors of the second image
	std::vector<bool> checkMatches(keypoints2.size(), true);
	for (int i = 0; i < keypoints1.size(); ++i) {
		const Keypoint& kp1 = keypoints1[i];
		std::map<int, std::vector<int>>::iterator it_candidates = tabIndices.find((int)kp1.getX());
		if (it_candidates == tabIndices.end())
			continue;
		const std::vector<int>& candidates = it_candidates->second;
		int bestIdx;
		float minDist;
		if (findBestMatch(descriptors1.row(i), descriptors2, candidates, bestIdx, minDist) && checkMatches[bestIdx]) {
			checkMatches[bestIdx] = false;
			matches.push_back(DescriptorMatch(i, bestIdx, minDist));
		}
	}
	return FrameworkReturnCode::_SUCCESS;
}


}
}
}
