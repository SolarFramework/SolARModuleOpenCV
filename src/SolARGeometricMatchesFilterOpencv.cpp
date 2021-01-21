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

#include "SolAROpenCVHelper.h"

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
    declareProperty("lineParallelismThreshold", m_lineParallelismThreshold);
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
    outputMatches.swap(tempMatches);
    return;
}

void SolARGeometricMatchesFilterOpencv::filter(const std::vector<DescriptorMatch> & inputMatches,
											   std::vector<DescriptorMatch> & outputMatches,
											   const std::vector<Keyline> & inputKeylinesA,
											   const std::vector<Keyline> & inputKeylinesB)
{
	if (inputMatches.empty())
		return;

	std::vector<DescriptorMatch> tempMatches;
	std::vector<uchar> status(inputKeylinesA.size());
	std::vector<cv::Point2f> pts1, pts2;

	for (const auto& match : inputMatches)
	{
		assert(match.getIndexInDescriptorA() < inputKeylinesA.size());
		pts1.push_back(cv::Point2f(inputKeylinesA[match.getIndexInDescriptorA()].getX(),
			inputKeylinesA[match.getIndexInDescriptorA()].getY()));

		assert(match.getIndexInDescriptorB() < inputKeylinesB.size());
		pts2.push_back(cv::Point2f(inputKeylinesB[match.getIndexInDescriptorB()].getX(),
			inputKeylinesB[match.getIndexInDescriptorB()].getY()));
	}

	{
		double minVal, maxVal;
		cv::minMaxIdx(pts1, &minVal, &maxVal);
		cv::Mat F = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, m_outlierDistanceRatio * maxVal, m_confidence, status);
	}

	for (int i = 0; i < status.size(); i++)
		if (status[i])
			tempMatches.push_back(inputMatches[i]);

	outputMatches.swap(tempMatches);
	return;
}

void SolARGeometricMatchesFilterOpencv::filter(const std::vector<DescriptorMatch>& inputMatches, std::vector<DescriptorMatch>& outputMatches, const std::vector<Keypoint>& inputKeyPoints1, const std::vector<Keypoint>& inputKeyPoints2, const Transform3Df & pose1, const Transform3Df & pose2, const CamCalibration & intrinsicParams)
{
	std::vector<DescriptorMatch> tmpMatches;
	// Compute essential matrix
	cv::Mat K(3, 3, CV_32FC1, (void *)intrinsicParams.data());
	cv::Mat E12 = K.t().inv() * SolAROpenCVHelper::computeFundamentalMatrix(pose1, pose2) * K.inv();
	
	// Check matches based on distance to epipolar lines
	for (int i = 0; i < inputMatches.size(); ++i)
	{
		Keypoint kp1 = inputKeyPoints1[inputMatches[i].getIndexInDescriptorA()];
		Keypoint kp2 = inputKeyPoints2[inputMatches[i].getIndexInDescriptorB()];
		// Epipolar line in second image l = x1'E12 = [a b c]
		double a = kp1.getX() * E12.at<float>(0, 0) + kp1.getY() * E12.at<float>(1, 0) + E12.at<float>(2, 0);
		double b = kp1.getX() * E12.at<float>(0, 1) + kp1.getY() * E12.at<float>(1, 1) + E12.at<float>(2, 1);
		double c = kp1.getX() * E12.at<float>(0, 2) + kp1.getY() * E12.at<float>(1, 2) + E12.at<float>(2, 2);
		double dis = std::fabs(a * kp2.getX() + b * kp2.getY() + c) / (std::sqrt(a * a + b * b));

		if (dis < m_epilinesDistance)
			tmpMatches.push_back(inputMatches[i]);
	}
	outputMatches.swap(tmpMatches);
}

void SolARGeometricMatchesFilterOpencv::filter(const std::vector<datastructure::DescriptorMatch> & inputMatches, std::vector<datastructure::DescriptorMatch> & outputMatches, const std::vector<datastructure::Keyline> & inputKeylines1, const std::vector<datastructure::Keyline> & inputKeylines2, const datastructure::Transform3Df & pose1, const datastructure::Transform3Df & pose2, const datastructure::CamCalibration & intrinsicParams)
{
	std::vector<DescriptorMatch> tmpMatches;
	// Compute essential matrix
	cv::Mat K(3, 3, CV_32FC1, (void *)intrinsicParams.data());
	cv::Mat E12 = K.t().inv() * SolAROpenCVHelper::computeFundamentalMatrix(pose1, pose2) * K.inv();
	// Classic approach used for keypoints must be adapted here,
	// as two keylines do not necessarily share the same endpoints
	for (const auto& match : inputMatches)
	{
		Keyline kl1 = inputKeylines1[match.getIndexInDescriptorA()];
		Keyline kl2 = inputKeylines2[match.getIndexInDescriptorB()];
		// Get start and end points in homogeneous space
		cv::Mat start1	= (cv::Mat_<float>(3, 1) << kl1.getStartPointX(),	kl1.getStartPointY(),	1.f);
		cv::Mat end1	= (cv::Mat_<float>(3, 1) << kl1.getEndPointX(),		kl1.getEndPointY(),		1.f);
		cv::Mat start2	= (cv::Mat_<float>(3, 1) << kl2.getStartPointX(),	kl2.getStartPointY(),	1.f);
		cv::Mat end2	= (cv::Mat_<float>(3, 1) << kl2.getEndPointX(),		kl2.getEndPointY(),		1.f);
		// Define l1 and l2 Plucker coordinates
		cv::Mat l1 = start1.cross(end1);
		cv::Mat l2 = start2.cross(end2);

		// Assert that epipolar lines passing trough l1 endpoints are not parallel to l2 and respectively
		cv::Mat epipolarLine_start1 = start1.t() * E12;
		cv::Mat epipolarLine_end1	= end1.t() * E12;
		cv::Mat epipolarLine_start2 = E12 * start2;
		cv::Mat epipolarLine_end2	= E12 * end2;
		auto checkParallel = [&](const auto& l1, const auto& l2) -> bool
		{
			return std::abs(
					l1.at<float>(0) * l2.at<float>(1) - l1.at<float>(1) * l2.at<float>(0)
				   ) < m_lineParallelismThreshold;
		};
		if ( checkParallel(epipolarLine_start1, l2)
		  || checkParallel(epipolarLine_end1, 	l2)
		  || checkParallel(epipolarLine_start2, l1)
		  || checkParallel(epipolarLine_end2, 	l1) )
		  	break;

		// Compute epipolar line passing through middle1 in second image m1'E12 = [a b c]
		cv::Mat middle1 = (cv::Mat_<float>(3, 1) << kl1.getX(), kl1.getY(), 1.f);
		cv::Mat middle2 = (cv::Mat_<float>(3, 1) << kl2.getX(), kl2.getY(), 1.f);
		cv::Mat epipolarLine = middle1.t() * E12;
		float dist =
			std::abs(epipolarLine.dot(middle2.t())) /
			std::sqrt(std::pow(epipolarLine.at<float>(0), 2.f) +
					  std::pow(epipolarLine.at<float>(1), 2.f));

		if (dist < m_epilinesDistance)
			tmpMatches.push_back(match);
	}
	outputMatches.swap(tmpMatches);
}

}
}
}
