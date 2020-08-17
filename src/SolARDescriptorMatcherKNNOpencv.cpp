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

	IDescriptorMatcher::RetCode SolARDescriptorMatcherKNNOpencv::matchInRegion(const std::vector<Point2Df>& points2D, const std::vector<SRef<DescriptorBuffer>>& descriptors, const SRef<Frame> frame, std::vector<DescriptorMatch>& matches, const float radius)
	{

		matches.clear();

		SRef<DescriptorBuffer> descriptorFrame = frame->getDescriptors();

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
		std::vector<cv::Point2f> pointsForSearch;
		for (auto &it : keypointsFrame)
			pointsForSearch.push_back(cv::Point2f(it.getX(), it.getY()));

		cv::flann::KDTreeIndexParams indexParams;
		cv::flann::Index kdtree(cv::Mat(pointsForSearch).reshape(1), indexParams);

		int imgWidth = frame->getView()->getWidth();
		int imgHeight = frame->getView()->getHeight();
		
		// Match each descriptor to descriptors of frame in a corresponding region
		std::vector<bool> checkMatches(keypointsFrame.size(), true);
		int idx = 0;
		for (auto &it : cvDescriptors) {
			std::vector<int> idxCandidates;
			if ((points2D[idx].getX() > 0) && (points2D[idx].getX() < imgWidth) && (points2D[idx].getY() > 0) && (points2D[idx].getY() < imgHeight)) {
				std::vector<float> dists;
				std::vector<float> query = { points2D[idx].getX(), points2D[idx].getY() };
				kdtree.radiusSearch(query, idxCandidates, dists, radius, 10);
			}

			if (idxCandidates.size() > 1) {
				float bestDist = std::numeric_limits<float>::max();
				float bestDist2 = std::numeric_limits<float>::max();
				int bestIdx = -1;
				for (auto &it_des: idxCandidates) {
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

				if ((bestIdx != -1) && (bestDist < m_distanceRatio * bestDist2) && (checkMatches[bestIdx])) {
					matches.push_back(DescriptorMatch(idx, bestIdx, bestDist));
					checkMatches[bestIdx] = false;
				}
			}

			idx++;
		}		

		return IDescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK;
	}

    }
    }
}  // end of namespace SolAR
