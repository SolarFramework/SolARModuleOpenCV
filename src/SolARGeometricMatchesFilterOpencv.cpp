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

#include "SolARGeometricMatchesFilterOpencv.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARGeometricMatchesFilterOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARGeometricMatchesFilterOpencv::SolARGeometricMatchesFilterOpencv():ConfigurableBase(xpcf::toUUID<SolARGeometricMatchesFilterOpencv>())
{ 
    LOG_DEBUG("SolARGeometricMatchesFilterOpencv constructor")
    declareInterface<api::features::IMatchesFilter>(this);
    declareProperty("confidence", m_confidence);
    declareProperty("outlierDistanceRatio", m_outlierDistanceRatio);
}


SolARGeometricMatchesFilterOpencv::~SolARGeometricMatchesFilterOpencv(){

}



void SolARGeometricMatchesFilterOpencv::filter(const std::vector<DescriptorMatch> & inputMatches,
                                               std::vector<DescriptorMatch> & outputMatches,
                                               const std::vector<Keypoint> & inputKeyPointsA,
                                               const std::vector<Keypoint> & inputKeyPointsB)
{

    std::vector<DescriptorMatch>tempMatches;
    std::vector<uchar> status(inputKeyPointsA.size());
    std::vector<cv::Point2f> pts1, pts2;

    if(inputMatches.size()){

        // get Align matches
        for (unsigned int i = 0; i<inputMatches.size(); i++) {
            assert(inputMatches[i].getIndexInDescriptorA() < inputKeyPointsA.size());
            pts1.push_back(cv::Point2f(inputKeyPointsA[inputMatches[i].getIndexInDescriptorA()].getX(),
                                       inputKeyPointsA[inputMatches[i].getIndexInDescriptorA()].getY()));

            assert(inputMatches[i].getIndexInDescriptorB() < inputKeyPointsB.size());
            pts2.push_back(cv::Point2f(inputKeyPointsB[inputMatches[i].getIndexInDescriptorB()].getX(),
                                       inputKeyPointsB[inputMatches[i].getIndexInDescriptorB()].getY()));

        }


        cv::Mat F;
        {
            double minVal, maxVal;
            cv::minMaxIdx(pts1, &minVal, &maxVal);
            F = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, m_outlierDistanceRatio * maxVal, m_confidence, status);
        }

        for (unsigned int i = 0; i<status.size(); i++) {
            if (status[i]) {
                   tempMatches.push_back(inputMatches[i]);
            }
        }
    }
    outputMatches = tempMatches;
    return;
}

}
}
}
