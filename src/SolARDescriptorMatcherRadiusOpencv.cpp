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
#include <iostream>
#include <utility>

#include "SolAROpenCVHelper.h"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorMatcherRadiusOpencv);

namespace SolAR {
using namespace datastructure;
using namespace api::features;
namespace MODULES {
namespace OPENCV {

SolARDescriptorMatcherRadiusOpencv::SolARDescriptorMatcherRadiusOpencv()
{
    setUUID(SolARDescriptorMatcherRadiusOpencv::UUID);
    addInterface<IDescriptorMatcher>(this,IDescriptorMatcher::UUID, "interface IDescriptorMatcher");
    LOG_DEBUG(" SolARDescriptorMatcherRadiusOpencv constructor")
    m_maxDistance = 1.0f;
}

SolARDescriptorMatcherRadiusOpencv::~SolARDescriptorMatcherRadiusOpencv()
{
    LOG_DEBUG(" SolARDescriptorMatcherRadiusOpencv destructor")
}

DescriptorMatcher::RetCode SolARDescriptorMatcherRadiusOpencv::match(
            SRef<DescriptorBuffer>& desc1,
            SRef<DescriptorBuffer>& desc2,
            std::vector<DescriptorMatch>& matches)
    {
 
        // check if the descriptors type match
        
        if (desc1->getDescriptorDataType() != desc2->getDescriptorDataType() ){
            return DescriptorMatcher::DESCRIPTORS_DONT_MATCH;
        }
 
        if (desc1->getNbDescriptors() == 0 || desc2->getNbDescriptors() == 0)
            return DescriptorMatcher::RetCode::DESCRIPTOR_EMPTY;
 
 
        std::vector<std::vector<cv::DMatch>> cv_matches;
        std::vector<cv::DMatch> good_matches;
 
        //since it is an openCV implementation we need to convert back the descriptors from SolAR to Opencv
        uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(desc1->getDescriptorDataType());

        cv::Mat cvDescriptors1(desc1->getNbDescriptors(), desc1->getNbElements(), type_conversion);
        cvDescriptors1.data=(uchar*)desc1->data();
 
        cv::Mat cvDescriptors2(desc2->getNbDescriptors(), desc2->getNbElements(), type_conversion);
        cvDescriptors2.data=(uchar*)desc2->data();

        if (desc1->getDescriptorDataType() != DescriptorBuffer::TYPE_32F){
            cvDescriptors1.convertTo(cvDescriptors1, CV_32F);
        }
        if (desc2->getDescriptorDataType() != DescriptorBuffer::TYPE_32F){
            cvDescriptors2.convertTo(cvDescriptors2, CV_32F);
        }

        m_matcher.radiusMatch(cvDescriptors1, cvDescriptors2, cv_matches, m_maxDistance);
 
        matches.clear();
        for (std::vector<std::vector<cv::DMatch>>::iterator itr=cv_matches.begin();itr!=cv_matches.end();++itr){
            for (std::vector<cv::DMatch>::iterator jtr = itr->begin(); jtr != itr->end(); ++jtr){
            
                matches.push_back(DescriptorMatch(jtr->queryIdx, jtr->trainIdx,jtr->distance ));
          
            }
        }

     if (matches.size()>0)
        return DescriptorMatcher::DESCRIPTORS_MATCHER_OK;
     else
         return DescriptorMatcher::DESCRIPTORS_DONT_MATCH;
                
    }
 
 DescriptorMatcher::RetCode SolARDescriptorMatcherRadiusOpencv::match(
           SRef<DescriptorBuffer>& descriptors1,
           std::vector<SRef<DescriptorBuffer>>& descriptors2,
           std::vector<DescriptorMatch>& matches
    )
    {
        if (descriptors1->getNbDescriptors() ==0 || descriptors2.size()== 0){
            return DescriptorMatcher::RetCode::DESCRIPTOR_EMPTY;
        }
 
        uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptors1->getDescriptorDataType());
 
        cv::Mat cvDescriptors1(descriptors1->getNbDescriptors(), descriptors1->getNbElements(), type_conversion);
        cvDescriptors1.data=(uchar*)descriptors1->data();
 
        if (descriptors1->getDescriptorDataType() != DescriptorBuffer::TYPE_32F){
            cvDescriptors1.convertTo(cvDescriptors1, CV_32F);
        }
 
        std::vector<cv::Mat> cvDescriptors;
        for(unsigned k=0;k<descriptors2.size();k++){
 
            uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptors2[k]->getDescriptorDataType());
 
            cv::Mat cvDescriptor(descriptors2[k]->getNbDescriptors(), descriptors2[k]->getNbElements(), type_conversion);
            cvDescriptor.data=(uchar*)(descriptors2[k]->data());
 
            if (descriptors2[k]->getDescriptorDataType() != DescriptorBuffer::TYPE_32F){
                cvDescriptor.convertTo(cvDescriptor, CV_32F);
            }
        
             cvDescriptors.push_back(cvDescriptor);
        }
 
 
        cv::Mat cvDescriptors2;
        cv::vconcat(cvDescriptors,cvDescriptors2);
 
        int nbOfMatches=1;
 
        if(cvDescriptors2.rows<nbOfMatches)
            return DescriptorMatcher::DESCRIPTORS_MATCHER_OK;
 
        std::vector<std::vector<cv::DMatch>> cv_matches;
 
        m_matcher.radiusMatch(cvDescriptors1, cvDescriptors2, cv_matches, m_maxDistance);
 
        matches.clear();
        for (std::vector<std::vector<cv::DMatch>>::iterator itr=cv_matches.begin();itr!=cv_matches.end();++itr)
        {
            for (std::vector<cv::DMatch>::iterator jtr = itr->begin(); jtr != itr->end(); ++jtr)
            {
                matches.push_back(DescriptorMatch(jtr->queryIdx, jtr->trainIdx,jtr->distance ));
            }
        }
 
        if (matches.size()>0)
            return DescriptorMatcher::DESCRIPTORS_MATCHER_OK;
        else
            return DescriptorMatcher::DESCRIPTORS_DONT_MATCH;
 
    }
 
}
}
}  // end of namespace SolAR
