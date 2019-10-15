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
    declareProperty("epilinesDistance", m_epilinesDistance);
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

void SolARGeometricMatchesFilterOpencv::filter(const std::vector<DescriptorMatch>& inputMatches, std::vector<DescriptorMatch>& outputMatches, const std::vector<Keypoint>& inputKeyPoints1, const std::vector<Keypoint>& inputKeyPoints2, const Transform3Df & pose1, const Transform3Df & pose2, const CamCalibration & intrinsicParams)
{
	std::vector<DescriptorMatch> tmpMatches;
	// compute fundamental matrice
	Transform3Df pose12 = pose1.inverse() * pose2;
	cv::Mat R12 = (cv::Mat_<float>(3, 3) << pose12(0, 0), pose12(0, 1), pose12(0, 2),
											pose12(1, 0), pose12(1, 1), pose12(1, 2), 
											pose12(2, 0), pose12(2, 1), pose12(2, 2));
	cv::Mat T12 = (cv::Mat_<float>(3, 1) << pose12(0, 3), pose12(1, 3), pose12(2, 3));

	cv::Mat T12x = (cv::Mat_<float>(3, 3) << 0, -T12.at<float>(2), T12.at<float>(1),
											T12.at<float>(2), 0, -T12.at<float>(0),
											-T12.at<float>(1), T12.at<float>(0), 0);

	cv::Mat K(3, 3, CV_32FC1, (void *)intrinsicParams.data());		
	cv::Mat F12 = K.t().inv() * T12x * R12 * K.inv();
	
	// check matches based on distance to epipolar lines
	for (int i = 0; i < inputMatches.size(); ++i) {		
		Keypoint kp1 = inputKeyPoints1[inputMatches[i].getIndexInDescriptorA()];
		Keypoint kp2 = inputKeyPoints2[inputMatches[i].getIndexInDescriptorB()];
		// Epipolar line in second image l = x1'F12 = [a b c]
		double a = kp1.getX() * F12.at<float>(0, 0) + kp1.getY() * F12.at<float>(1, 0) + F12.at<float>(2, 0);
		double b = kp1.getX() * F12.at<float>(0, 1) + kp1.getY() * F12.at<float>(1, 1) + F12.at<float>(2, 1);
		double c = kp1.getX() * F12.at<float>(0, 2) + kp1.getY() * F12.at<float>(1, 2) + F12.at<float>(2, 2);
		double dis = std::fabs(a * kp2.getX() + b * kp2.getY() + c) / (std::sqrt(a * a + b * b));

		if (dis < m_epilinesDistance)
			tmpMatches.push_back(inputMatches[i]);
	}
	outputMatches.swap(tmpMatches);
}

}
}
}
