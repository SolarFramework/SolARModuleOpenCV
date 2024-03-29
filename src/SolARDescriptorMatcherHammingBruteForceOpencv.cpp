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

#include "SolARDescriptorMatcherHammingBruteForceOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorMatcherHammingBruteForceOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARDescriptorMatcherHammingBruteForceOpencv::SolARDescriptorMatcherHammingBruteForceOpencv():base::features::ADescriptorMatcher(xpcf::toMap<SolARDescriptorMatcherHammingBruteForceOpencv>())
{
	declareProperty("distanceRatio", m_distanceRatio);
    LOG_DEBUG(" SolARDescriptorMatcherHammingBruteForceOpencv constructor")
}

SolARDescriptorMatcherHammingBruteForceOpencv::~SolARDescriptorMatcherHammingBruteForceOpencv()
{
    LOG_DEBUG(" SolARDescriptorMatcherHammingBruteForceOpencv destructor")
}

xpcf::XPCFErrorCode SolARDescriptorMatcherHammingBruteForceOpencv::onConfigured()
{
	LOG_DEBUG(" SolARDescriptorMatcherHammingBruteForceOpencv onConfigured");
#ifdef WITHCUDA
	m_matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
#else
	m_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::MatcherType::BRUTEFORCE_HAMMING);
#endif // WITHCUDA		
	return xpcf::XPCFErrorCode::_SUCCESS;
}

FrameworkReturnCode SolARDescriptorMatcherHammingBruteForceOpencv::match(SRef<DescriptorBuffer> desc1,
																		SRef<DescriptorBuffer> desc2, 
																		std::vector<DescriptorMatch>& matches)
{
	// check conditions
	if ((desc1->getDescriptorType() != desc2->getDescriptorType()) ||
		desc1->getNbDescriptors() == 0 || desc2->getNbDescriptors() == 0 || 
		desc1->getDescriptorDataType() != SolAR::DescriptorDataType::TYPE_8U) {
		return FrameworkReturnCode::_ERROR_;
	}
  
	//since it is an openCV implementation we need to convert back the descriptors from SolAR to Opencv
	uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(desc1->getDescriptorDataType());
 
	cv::Mat cvDescriptor1(desc1->getNbDescriptors(), desc1->getNbElements(), type_conversion);
	cvDescriptor1.data=(uchar*)desc1->data();
 
	cv::Mat cvDescriptor2(desc2->getNbDescriptors(), desc1->getNbElements(), type_conversion);
	cvDescriptor2.data=(uchar*)desc2->data();

	std::vector< std::vector<cv::DMatch> > nn_matches;
#ifdef WITHCUDA
	cv::cuda::GpuMat cvDescriptor1Gpu, cvDescriptor2Gpu;
	cvDescriptor1Gpu.upload(cvDescriptor1);
	cvDescriptor2Gpu.upload(cvDescriptor2);
	m_matcher->knnMatch(cvDescriptor2Gpu, cvDescriptor1Gpu, nn_matches, 2);
#else
	m_matcher->knnMatch(cvDescriptor2, cvDescriptor1, nn_matches, 2);
#endif // WITHCUDA
	std::map<uint32_t, std::map<uint32_t, float>> matches12;
	for (unsigned i = 0; i < nn_matches.size(); i++) {
		if (nn_matches[i][0].distance < m_distanceRatio * nn_matches[i][1].distance) {
			matches12[nn_matches[i][0].trainIdx][nn_matches[i][0].queryIdx] = nn_matches[i][0].distance;
		}
	}

	// get best matches to descriptors 1
	for (auto it_des1 : matches12) {
		uint32_t idxDes1 = it_des1.first;
		std::map<uint32_t, float> infoMatch = it_des1.second;
		uint32_t bestIdxDes2;
		float bestDistance = FLT_MAX;
		for (auto it_des2 : infoMatch)
			if (it_des2.second < bestDistance) {
				bestDistance = it_des2.second;
				bestIdxDes2 = it_des2.first;
			}
		matches.push_back(DescriptorMatch(idxDes1, bestIdxDes2, bestDistance));
	}
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}  // end of namespace SolAR
