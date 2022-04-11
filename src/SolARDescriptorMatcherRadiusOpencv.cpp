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

#include "SolARDescriptorMatcherRadiusOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorMatcherRadiusOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARDescriptorMatcherRadiusOpencv::SolARDescriptorMatcherRadiusOpencv(): base::features::ADescriptorMatcher(xpcf::toMap<SolARDescriptorMatcherRadiusOpencv>())
{
	declareProperty("maxDistance", m_maxDistance);
	declareProperty("type", m_type);
    LOG_DEBUG(" SolARDescriptorMatcherRadiusOpencv constructor")
}

SolARDescriptorMatcherRadiusOpencv::~SolARDescriptorMatcherRadiusOpencv()
{
    LOG_DEBUG(" SolARDescriptorMatcherRadiusOpencv destructor")
}

xpcf::XPCFErrorCode SolARDescriptorMatcherRadiusOpencv::onConfigured()
{
	LOG_DEBUG(" SolARDescriptorMatcherRadiusOpencv onConfigured");
#ifdef WITHCUDA
	m_matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L2);
#else
	if (SolAROpenCVHelper::createMatcher(m_type, m_matcher) != FrameworkReturnCode::_SUCCESS) {
		LOG_ERROR("Descriptor matcher type {} is not supported", m_type);
		return xpcf::XPCFErrorCode::_FAIL;
	}
#endif // WITHCUDA	
	return xpcf::XPCFErrorCode::_SUCCESS;
}

FrameworkReturnCode SolARDescriptorMatcherRadiusOpencv::match(
            SRef<DescriptorBuffer> desc1,
            SRef<DescriptorBuffer> desc2,
            std::vector<DescriptorMatch>& matches)
{
	matches.clear();
	// check conditions
	if ((desc1->getDescriptorType() != desc2->getDescriptorType()) ||
		desc1->getNbDescriptors() == 0 || desc2->getNbDescriptors() == 0) {
		return FrameworkReturnCode::_ERROR_;
	}

    std::vector<std::vector<cv::DMatch>> cv_matches;
    std::vector<cv::DMatch> good_matches;

    //since it is an openCV implementation we need to convert back the descriptors from SolAR to Opencv
    uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(desc1->getDescriptorDataType());

    cv::Mat cvDescriptors1(desc1->getNbDescriptors(), desc1->getNbElements(), type_conversion);
    cvDescriptors1.data=(uchar*)desc1->data();

    cv::Mat cvDescriptors2(desc2->getNbDescriptors(), desc2->getNbElements(), type_conversion);
    cvDescriptors2.data=(uchar*)desc2->data();

    if (desc1->getDescriptorDataType() != DescriptorDataType::TYPE_32F){
        cvDescriptors1.convertTo(cvDescriptors1, CV_32F);
    }
    if (desc2->getDescriptorDataType() != DescriptorDataType::TYPE_32F){
        cvDescriptors2.convertTo(cvDescriptors2, CV_32F);
    }

#ifdef WITHCUDA
	cv::cuda::GpuMat cvDescriptor1Gpu, cvDescriptor2Gpu;
	cvDescriptor1Gpu.upload(cvDescriptors1);
	cvDescriptor2Gpu.upload(cvDescriptors2);
	m_matcher->radiusMatch(cvDescriptor1Gpu, cvDescriptor2Gpu, cv_matches, m_maxDistance);
#else
	m_matcher->radiusMatch(cvDescriptors1, cvDescriptors2, cv_matches, m_maxDistance);
#endif // WITHCUDA	    

    matches.clear();
    for (std::vector<std::vector<cv::DMatch>>::iterator itr=cv_matches.begin();itr!=cv_matches.end();++itr){
        for (std::vector<cv::DMatch>::iterator jtr = itr->begin(); jtr != itr->end(); ++jtr){

            matches.push_back(DescriptorMatch(jtr->queryIdx, jtr->trainIdx,jtr->distance ));

        }
    }
	if (matches.size() > 0)
		return FrameworkReturnCode::_SUCCESS;
	else
		return FrameworkReturnCode::_ERROR_;
}

}
}
}  // end of namespace SolAR
