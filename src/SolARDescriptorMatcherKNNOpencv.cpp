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
//#include "SolARDescriptorOpencv.h"
#include <iostream>
#include <utility>

#include "SolAROpenCVHelper.h"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorMatcherKNNOpencv);

namespace SolAR {
using namespace datastructure;
using namespace api::features;
namespace MODULES {
namespace OPENCV {

SolARDescriptorMatcherKNNOpencv::SolARDescriptorMatcherKNNOpencv()
{
    setUUID(SolARDescriptorMatcherKNNOpencv::UUID);
    addInterface<IDescriptorMatcher>(this,IDescriptorMatcher::UUID, "interface SolARDescriptorMatcherKNNOpencv");
    LOG_DEBUG(" SolARDescriptorMatcherKNNOpencv constructor")
}

SolARDescriptorMatcherKNNOpencv::~SolARDescriptorMatcherKNNOpencv()
{
    LOG_DEBUG(" SolARDescriptorMatcherKNNOpencv destructor")
}

bool sortByDistance(const std::pair<int,float> &lhs, const std::pair<int,float> &rhs)
{
    return lhs.second < rhs.second;
}

// filter matches : keep the best match in case of multiple matches per keypoint
void filterMatches(std::vector<cv::DMatch>& matches){

    std::map<int,std::vector<std::pair<int,float>>> matchesMap;
    for(std::vector<cv::DMatch>::iterator itr=matches.begin();itr!=matches.end();++itr){
        matchesMap[itr->trainIdx].push_back(std::make_pair(itr->queryIdx,itr->distance));
    }

    matches.clear();
    for (std::map<int,std::vector<std::pair<int,float>>>::iterator itr=matchesMap.begin();itr!=matchesMap.end();++itr){
        std::vector<std::pair<int,float>> ptr=itr->second;
        if(ptr.size()>1){

            std::sort(ptr.begin(),ptr.end(),sortByDistance);
        }

        cv::DMatch dm;
        dm.trainIdx=itr->first;
        dm.queryIdx=ptr.begin()->first;
        dm.distance=ptr.begin()->second;
        matches.push_back(dm);
    }


    matchesMap.clear();
    for(std::vector<cv::DMatch>::iterator itr=matches.begin();itr!=matches.end();++itr){
        matchesMap[itr->queryIdx].push_back(std::make_pair(itr->trainIdx,itr->distance));
    }

    matches.clear();
    for (std::map<int,std::vector<std::pair<int,float>>>::iterator itr=matchesMap.begin();itr!=matchesMap.end();++itr){
        std::vector<std::pair<int,float>> ptr=itr->second;
        if(ptr.size()>1){
            std::sort(ptr.begin(),ptr.end(),sortByDistance);
        }
        cv::DMatch dm;
        dm.queryIdx=itr->first;
        dm.trainIdx=ptr.begin()->first;
        dm.distance=ptr.begin()->second;
        matches.push_back(dm);
    }

    LOG_INFO("number of matches : {}",matches.size());
}


void keepGoodMAtches(std::vector<std::vector<cv::DMatch>> &matches,std::vector<cv::DMatch>& good_matches ){

    good_matches.clear();

    std::vector<cv::DMatch>::iterator jtr,jtr1;
    for (std::vector<std::vector<cv::DMatch>>::iterator itr=matches.begin();itr!=matches.end();++itr){
        jtr=itr->begin();
        if(itr->size()>1){
            jtr1=jtr+1;
            if( jtr->distance < 0.75*jtr1->distance) {
                cv::DMatch dm;
                dm.queryIdx=jtr->queryIdx;
                dm.trainIdx=jtr->trainIdx;
                dm.distance=jtr->distance;
                good_matches.push_back(dm);
            }
        }
        else {
            cv::DMatch dm;
            dm.queryIdx=jtr->queryIdx;
            dm.trainIdx=jtr->trainIdx;
            dm.distance=jtr->distance;
            good_matches.push_back(dm);
        }
    }
   filterMatches(good_matches);
}



DescriptorMatcher::RetCode SolARDescriptorMatcherKNNOpencv::match(SRef<DescriptorBuffer>& descriptors1,SRef<DescriptorBuffer>& descriptors2,std::vector<std::vector< cv::DMatch >>& matches,int nbOfMatches)
{
    matches.clear();

    // check if the descriptors type match
    if (descriptors1->getDescriptorDataType() != descriptors2->getDescriptorDataType() ){
            return DescriptorMatcher::DESCRIPTORS_DONT_MATCH;
     }

    if(descriptors1->getNbDescriptors()==0 || descriptors2->getNbDescriptors()==0){
        return DescriptorMatcher::DESCRIPTOR_EMPTY;
    }

    // to prevent pb in knn search
    nbOfMatches=cv::min(nbOfMatches,(int)descriptors2->getNbDescriptors());

    //since it is an openCV implementation we need to convert back the descriptors from SolAR to Opencv

    uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptors1->getDescriptorDataType());

    cv::Mat cvDescriptor1(descriptors1->getNbDescriptors(), descriptors1->getNbElements(), type_conversion);
    cvDescriptor1.data=(uchar*)descriptors1->data();

    cv::Mat cvDescriptor2(descriptors2->getNbDescriptors(), descriptors2->getNbElements(), type_conversion);
    cvDescriptor2.data=(uchar*)descriptors2->data();
/*
    if (descriptors1->getDescriptorDataType() != DescriptorBuffer::TYPE_32F)
       cvDescriptor1.convertTo(cvDescriptor1, CV_32F);
    if (descriptors2->getDescriptorDataType() != DescriptorBuffer::TYPE_32F)
       cvDescriptor2.convertTo(cvDescriptor2, CV_32F);
*/
    m_matcher.knnMatch(cvDescriptor1, cvDescriptor2, matches,nbOfMatches);

    return DescriptorMatcher::DESCRIPTORS_MATCHER_OK;

}

DescriptorMatcher::RetCode SolARDescriptorMatcherKNNOpencv::match(
        sptrnms::shared_ptr<DescriptorBuffer>& desc1, sptrnms::shared_ptr<DescriptorBuffer>& desc2, std::vector<DescriptorMatch>& matches){
 
    // check if the descriptors type match
    if(desc1->getDescriptorType() != desc2->getDescriptorType()){
        return DescriptorMatcher::DESCRIPTORS_DONT_MATCH;
    }
    if (desc1->getNbDescriptors() == 0 || desc2->getNbDescriptors() == 0)
        return DescriptorMatcher::RetCode::DESCRIPTOR_EMPTY;
 
    std::vector<std::vector<cv::DMatch>> initial_matches;
    std::vector<cv::DMatch> good_matches;
 
     //since it is an openCV implementation we need to convert back the descriptors from SolAR to Opencv
     uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(desc1->getDescriptorDataType());
 
     cv::Mat cvDescriptor1(desc1->getNbDescriptors(), desc1->getNbElements(), type_conversion);
     cvDescriptor1.data=(uchar*)desc1->data();
 
     cv::Mat cvDescriptor2(desc2->getNbDescriptors(), desc1->getNbElements(), type_conversion);
     cvDescriptor2.data=(uchar*)desc2->data();
 
     if (desc1->getDescriptorDataType() != DescriptorBuffer::TYPE_32F)
        cvDescriptor1.convertTo(cvDescriptor1, CV_32F);
     if (desc2->getDescriptorDataType() != DescriptorBuffer::TYPE_32F)
        cvDescriptor2.convertTo(cvDescriptor2, CV_32F);
 
     m_matcher.knnMatch(cvDescriptor1, cvDescriptor2, initial_matches,2);
 
     keepGoodMAtches(initial_matches,good_matches);
 
     matches.clear();
     for(auto currentMatch : good_matches) {
            matches.push_back(DescriptorMatch(currentMatch.queryIdx, currentMatch.trainIdx,currentMatch.distance ));
     }
 
     return DescriptorMatcher::DESCRIPTORS_MATCHER_OK;
 
}
 
DescriptorMatcher::RetCode SolARDescriptorMatcherKNNOpencv::match(
       SRef<DescriptorBuffer>& descriptors1,
       std::vector<SRef<DescriptorBuffer>>& descriptors2,
        std::vector<DescriptorMatch>& matches
        ){
 
 
    if (descriptors1->getNbDescriptors() ==0 || descriptors2.size()== 0)
        return DescriptorMatcher::RetCode::DESCRIPTOR_EMPTY;
 
    uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptors1->getDescriptorDataType());
 
    cv::Mat cvDescriptors1(descriptors1->getNbDescriptors(), descriptors1->getNbElements(), type_conversion);
    cvDescriptors1.data=(uchar*)descriptors1->data();
 
    if (descriptors1->getDescriptorDataType() != DescriptorBuffer::TYPE_32F)
       cvDescriptors1.convertTo(cvDescriptors1, CV_32F);
 
    std::vector<cv::Mat> cvDescriptors;
    for(unsigned k=0;k<descriptors2.size();k++){
 
        uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptors2[k]->getDescriptorDataType());
 
        cv::Mat cvDescriptor(descriptors2[k]->getNbDescriptors(), descriptors2[k]->getNbElements(), type_conversion);
        cvDescriptor.data=(uchar*)(descriptors2[k]->data());
 
        if (descriptors2[k]->getDescriptorDataType() != DescriptorBuffer::TYPE_32F)
           cvDescriptor.convertTo(cvDescriptor, CV_32F);
 
        cvDescriptors.push_back(cvDescriptor);
    }
 
 
    cv::Mat cvDescriptors2;
    cv::vconcat(cvDescriptors,cvDescriptors2);
 
 
    int nbOfMatches=1;
 
    if(cvDescriptors2.rows<nbOfMatches)
        return DescriptorMatcher::DESCRIPTORS_MATCHER_OK;
 
    std::vector<std::vector<cv::DMatch>> initial_matches;
    std::vector<cv::DMatch> good_matches;
 
    m_matcher.knnMatch(cvDescriptors1, cvDescriptors2, initial_matches,nbOfMatches);
 
    keepGoodMAtches(initial_matches,good_matches);
 
    matches.clear();
    for(auto currentMatch : good_matches) {
        
        matches.push_back(DescriptorMatch(currentMatch.queryIdx, currentMatch.trainIdx,currentMatch.distance ));
       
    }
 
    return DescriptorMatcher::DESCRIPTORS_MATCHER_OK;
 
}

}
}
}  // end of namespace SolAR
