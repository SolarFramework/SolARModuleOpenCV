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
    using namespace api::features;
    namespace MODULES {
    namespace OPENCV {

    SolARDescriptorMatcherKNNOpencv::SolARDescriptorMatcherKNNOpencv():ConfigurableBase(xpcf::toUUID<SolARDescriptorMatcherKNNOpencv>())
    {
        declareInterface<IDescriptorMatcher>(this);
        declareProperty("distanceRatio", m_distanceRatio);
        LOG_DEBUG(" SolARDescriptorMatcherKNNOpencv constructor")
    }

    SolARDescriptorMatcherKNNOpencv::~SolARDescriptorMatcherKNNOpencv()
    {
        LOG_DEBUG(" SolARDescriptorMatcherKNNOpencv destructor")
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

        if (desc1->getDescriptorDataType() != DescriptorBuffer::TYPE_32F)
            cvDescriptor1.convertTo(cvDescriptor1, CV_32F);
        if (desc2->getDescriptorDataType() != DescriptorBuffer::TYPE_32F)
            cvDescriptor2.convertTo(cvDescriptor2, CV_32F);

        std::vector< std::vector<cv::DMatch> > nn_matches;
        m_matcher.knnMatch(cvDescriptor1, cvDescriptor2, nn_matches,2);

        for(auto & nn_matche : nn_matches) {
            if(nn_matche[0].distance < m_distanceRatio * nn_matche[1].distance) {
                matches.emplace_back(nn_matche[0].queryIdx, nn_matche[0].trainIdx,nn_matche[0].distance);
            }
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

        if (descriptors1->getNbDescriptors() ==0 || descriptors2.empty())
            return IDescriptorMatcher::RetCode::DESCRIPTOR_EMPTY;

        uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptors1->getDescriptorDataType());

        cv::Mat cvDescriptors1(descriptors1->getNbDescriptors(), descriptors1->getNbElements(), type_conversion);
        cvDescriptors1.data=(uchar*)descriptors1->data();

        if (descriptors1->getDescriptorDataType() != DescriptorBuffer::TYPE_32F)
            cvDescriptors1.convertTo(cvDescriptors1, CV_32F);

        std::vector<cv::Mat> cvDescriptors;
        for(const auto & k : descriptors2){

            uint32_t type_conversion= SolAROpenCVHelper::deduceOpenDescriptorCVType(k->getDescriptorDataType());

            cv::Mat cvDescriptor(k->getNbDescriptors(), k->getNbElements(), type_conversion);
            cvDescriptor.data=(uchar*)(k->data());

            if (k->getDescriptorDataType() != DescriptorBuffer::TYPE_32F)
                cvDescriptor.convertTo(cvDescriptor, CV_32F);

            cvDescriptors.emplace_back(cvDescriptor);
        }


        cv::Mat cvDescriptors2;
        cv::vconcat(cvDescriptors,cvDescriptors2);


        int nbOfMatches=1;

        if(cvDescriptors2.rows<nbOfMatches)
            return IDescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK;

        std::vector<std::vector<cv::DMatch>> initial_matches;
        std::vector<cv::DMatch> good_matches;

        std::vector< std::vector<cv::DMatch> > nn_matches;
        m_matcher.knnMatch(cvDescriptors1, cvDescriptors2, initial_matches,nbOfMatches);

        for(auto & nn_matche : nn_matches) {
            if(nn_matche[0].distance < m_distanceRatio * nn_matche[1].distance) {
                matches.emplace_back(nn_matche[0].queryIdx, nn_matche[0].trainIdx,nn_matche[0].distance);
            }
        }

        return IDescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK;

    }

    }
    }
}  // end of namespace SolAR
