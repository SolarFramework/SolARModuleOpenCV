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

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorMatcherKNNOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARDescriptorMatcherKNNOpencv::SolARDescriptorMatcherKNNOpencv(): base::features::ADescriptorMatcher(xpcf::toMap<SolARDescriptorMatcherKNNOpencv>())
{
    declareProperty("distanceRatio", m_distanceRatio);
    LOG_DEBUG(" SolARDescriptorMatcherKNNOpencv constructor")
}

SolARDescriptorMatcherKNNOpencv::~SolARDescriptorMatcherKNNOpencv()
{
    LOG_DEBUG(" SolARDescriptorMatcherKNNOpencv destructor")
}

FrameworkReturnCode SolARDescriptorMatcherKNNOpencv::match(SRef<DescriptorBuffer> desc1,
                                                        SRef<DescriptorBuffer> desc2,
                                                        std::vector<DescriptorMatch>& matches)
{
    matches.clear();

    // check conditions
    if ((desc1->getDescriptorType() != desc2->getDescriptorType()) ||
		desc1->getNbDescriptors() == 0 || desc2->getNbDescriptors() == 0 ||
		desc2->getNbDescriptors() < 2){
        return FrameworkReturnCode::_ERROR_;
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

    return FrameworkReturnCode::_SUCCESS;
}

}
}
}  // end of namespace SolAR
