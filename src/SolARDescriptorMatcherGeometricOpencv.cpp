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

#include "SolARDescriptorMatcherGeometricOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

// test optimization code implemented by yzhou 
#define OPTIM_ON

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorMatcherGeometricOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARDescriptorMatcherGeometricOpencv::SolARDescriptorMatcherGeometricOpencv(): base::features::ADescriptorMatcherGeometric(xpcf::toMap<SolARDescriptorMatcherGeometricOpencv>())
{
    declareProperty("distanceRatio", m_distanceRatio);
    declareProperty("paddingRatio", m_paddingRatio);
    declareProperty("matchingDistanceMax", m_matchingDistanceMax);
    LOG_DEBUG(" SolARDescriptorMatcherGeometricOpencv constructor")
}

SolARDescriptorMatcherGeometricOpencv::~SolARDescriptorMatcherGeometricOpencv()
{
    LOG_DEBUG(" SolARDescriptorMatcherGeometricOpencv destructor")
}

xpcf::XPCFErrorCode SolARDescriptorMatcherGeometricOpencv::onConfigured()
{
#ifdef OPTIM_ON
	LOG_DEBUG(" SolARDescriptorMatcherGeometricOpencv onConfigured");
#ifdef WITHCUDA
	m_matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L2);
#else
	if (SolAROpenCVHelper::createMatcher(m_type, m_matcher) != FrameworkReturnCode::_SUCCESS) {
		LOG_ERROR("Descriptor matcher type {} is not supported", m_type);
		return xpcf::XPCFErrorCode::_FAIL;
	}
#endif // WITHCUDA
#endif	
	return xpcf::XPCFErrorCode::_SUCCESS;
}

FrameworkReturnCode SolARDescriptorMatcherGeometricOpencv::match(const SRef<SolAR::datastructure::DescriptorBuffer> descriptors1,
                                                                 const SRef<SolAR::datastructure::DescriptorBuffer> descriptors2,
                                                                 const std::vector<SolAR::datastructure::Keypoint> &undistortedKeypoints1,
                                                                 const std::vector<SolAR::datastructure::Keypoint> &undistortedKeypoints2,
                                                                 const SolAR::datastructure::Transform3Df& pose1,
                                                                 const SolAR::datastructure::Transform3Df& pose2,
                                                                 const SolAR::datastructure::CameraParameters & camParams1,
                                                                 const SolAR::datastructure::CameraParameters & camParams2,
                                                                 std::vector<SolAR::datastructure::DescriptorMatch> & matches,
                                                                 const std::vector<uint32_t>& mask1,
																 const std::vector<uint32_t>& mask2)
{
	matches.clear();
	// check conditions
	if ((descriptors1->getDescriptorType() != descriptors2->getDescriptorType()) ||
		(descriptors1->getNbDescriptors() == 0) || (descriptors2->getNbDescriptors() == 0)) {
		return FrameworkReturnCode::_ERROR_;
	}
	// calculate fundametal matrix
#ifdef OPTIM_ON
	Transform3Df pose1Inv;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			pose1Inv(i, j) = pose1(j, i);
	for (int i = 0; i < 3; i++)
		pose1Inv(3, i) = 0.f;
	pose1Inv(3, 3) = 1.f;
	for (int i = 0; i < 3; i++)
		pose1Inv(i, 3) = -(pose1(0, 3)*pose1(0, i) + pose1(1, 3)*pose1(1, i) + pose1(2, 3)*pose1(2, i));
	Transform3Df pose12 = pose1Inv * pose2;
#else 
	Transform3Df pose12 = pose1.inverse() * pose2;
#endif
	cv::Mat R12 = (cv::Mat_<float>(3, 3) << pose12(0, 0), pose12(0, 1), pose12(0, 2),
		pose12(1, 0), pose12(1, 1), pose12(1, 2),
		pose12(2, 0), pose12(2, 1), pose12(2, 2));
	cv::Mat T12 = (cv::Mat_<float>(3, 1) << pose12(0, 3), pose12(1, 3), pose12(2, 3));
	cv::Mat T12x = (cv::Mat_<float>(3, 3) << 0, -T12.at<float>(2), T12.at<float>(1),
		T12.at<float>(2), 0, -T12.at<float>(0),
		-T12.at<float>(1), T12.at<float>(0), 0);
	cv::Mat K1(3, 3, CV_32FC1, (void *)camParams1.intrinsic.data());
	cv::Mat K2(3, 3, CV_32FC1, (void *)camParams2.intrinsic.data());
#ifdef OPTIM_ON
	cv::Mat K1tInv = cv::Mat::zeros(3, 3, CV_32FC1);
	K1tInv.at<float>(0, 0) = 1.f / K1.at<float>(0, 0);
	K1tInv.at<float>(1, 1) = 1.f / K1.at<float>(1, 1);
	K1tInv.at<float>(2, 2) = 1.f;
	K1tInv.at<float>(2, 0) = -K1.at<float>(0, 2) / K1.at<float>(0, 0);
	K1tInv.at<float>(2, 1) = -K1.at<float>(1, 2) / K1.at<float>(1, 1);
	cv::Mat K2Inv = cv::Mat::zeros(3, 3, CV_32FC1);
	K2Inv.at<float>(0, 0) = 1.f / K2.at<float>(0, 0);
	K2Inv.at<float>(1, 1) = 1.f / K2.at<float>(1, 1);
	K2Inv.at<float>(2, 2) = 1.f;
	K2Inv.at<float>(0, 2) = -K2.at<float>(0, 2) / K2.at<float>(0, 0);
	K2Inv.at<float>(1, 2) = -K2.at<float>(1, 2) / K2.at<float>(1, 1);
	cv::Mat F = K1tInv * T12x * R12 * K2Inv;
#else 
	cv::Mat F = K1.t().inv() * T12x * R12 * K2.inv();
#endif

	// get input points in the first frame
	std::vector<cv::Point2f> pts1;
	std::vector<uint32_t> indices1, indices2;
	if (mask1.size() == 0) {
		for (int i = 0; i < undistortedKeypoints1.size(); ++i) {
			pts1.push_back(cv::Point2f(undistortedKeypoints1[i].getX(), undistortedKeypoints1[i].getY()));
			indices1.push_back(i);
		}
	}
	else {
		indices1 = mask1;
		for (int i = 0; i < mask1.size(); ++i)
			pts1.push_back(cv::Point2f(undistortedKeypoints1[mask1[i]].getX(), undistortedKeypoints1[mask1[i]].getY()));
	}
	if (mask2.size() == 0) {
		for (int i = 0; i < undistortedKeypoints2.size(); ++i)
			indices2.push_back(i);
	}
	else {
		indices2 = mask2;
	}

	// compute epipolar lines
	std::vector<cv::Point3f> lines2;
	cv::computeCorrespondEpilines(pts1, 2, F, lines2);

	// convert frame descriptor to opencv's descriptor
	uint32_t type_conversion = SolAR::MODULES::OPENCV::SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptors1->getDescriptorDataType());
#ifdef OPTIM_ON
	cv::Mat cvDescriptor1(indices1.size(), descriptors1->getNbElements(), type_conversion);
	uchar* buf_in = (uchar*)descriptors1->data();
	uchar* buf_out = cvDescriptor1.data;
	uint32_t num_elements = static_cast<uint32_t>(descriptors1->getNbElements());
	for (auto i = 0; i < indices1.size(); i++) {
		std::memcpy(buf_out, buf_in + num_elements * indices1[i], num_elements);
		buf_out += num_elements;
	}
	cv::Mat cvDescriptor2(indices2.size(), descriptors2->getNbElements(), type_conversion);
	buf_in = (uchar*)descriptors2->data();
	buf_out = cvDescriptor2.data;
	num_elements = static_cast<uint32_t>(descriptors2->getNbElements());
	for (auto i = 0; i < indices2.size(); i++) {
		std::memcpy(buf_out, buf_in + num_elements * indices2[i], num_elements);
		buf_out += num_elements;
	}
#else 
	cv::Mat cvDescriptor1(descriptors1->getNbDescriptors(), descriptors1->getNbElements(), type_conversion);
	cvDescriptor1.data = (uchar*)descriptors1->data();
	cv::Mat cvDescriptor2(descriptors2->getNbDescriptors(), descriptors2->getNbElements(), type_conversion);
	cvDescriptor2.data = (uchar*)descriptors2->data();
#endif
	

#ifdef OPTIM_ON
	// perform descriptor matching first on GPU using Cuda if SolARModuleOpenCVCuda
	std::vector< std::vector<cv::DMatch> > nn_matches;
#ifdef WITHCUDA
	if (descriptors1->getDescriptorDataType() != DescriptorDataType::TYPE_32F) // convert to float because cuda descriptor match only supports float type 
		cvDescriptor1.convertTo(cvDescriptor1, CV_32F);
	if (descriptors2->getDescriptorDataType() != DescriptorDataType::TYPE_32F)
		cvDescriptor2.convertTo(cvDescriptor2, CV_32F);
	cv::cuda::GpuMat cvDescriptor1Gpu, cvDescriptor2Gpu;
	cvDescriptor1Gpu.upload(cvDescriptor1);
	cvDescriptor2Gpu.upload(cvDescriptor2);
	m_matcher->knnMatch(cvDescriptor1Gpu, cvDescriptor2Gpu, nn_matches, 2);
#else
	m_matcher->knnMatch(cvDescriptor1, cvDescriptor2, nn_matches, 2);
#endif
	
	// filter the matches by three conditions (best distance, 2nd best distance and epipolar constraint)
	std::vector<bool> checkMatches(indices2.size(), true);
	float acceptedDist = m_paddingRatio * camParams2.resolution.width;
	for (auto i = 0; i < indices1.size(); i++) {
		const auto& best_matches = nn_matches[i];
		if (!checkMatches[best_matches[0].trainIdx])
			continue;
		if (best_matches[0].distance >= m_matchingDistanceMax)
			continue;
		if (best_matches[0].distance >= m_distanceRatio * best_matches[1].distance) // no big difference between best and 2nd best, Lowe's ratio test
			continue;
		// the match is accepted 
		// now we should check the epipolar constraint 
		int idx1 = indices1[best_matches[0].queryIdx];
		int idx2 = indices2[best_matches[0].trainIdx];
		float distance = best_matches[0].distance;
		cv::Point3f l = lines2[i];  // line equation a*x + b*y +c = 0
		float x = undistortedKeypoints2[idx2].getX();
		float y = undistortedKeypoints2[idx2].getY();
		float disPointLine = std::abs(x * l.x + y * l.y + l.z) / std::sqrt(l.x * l.x + l.y * l.y);
		if (disPointLine < acceptedDist) {
			matches.push_back(DescriptorMatch(idx1, idx2, distance));
			checkMatches[best_matches[0].trainIdx] = false;
		}
	}
#else 
	// first apply epipolar constraint then apply descriptor matching  
	std::vector<bool> checkMatches(undistortedKeypoints2.size(), true);
	float acceptedDist = m_paddingRatio * camParams2.resolution.width;
	for (int i = 0; i < indices1.size(); i++) {
		const cv::Mat cvDes1 = cvDescriptor1.row(indices1[i]);
		float bestDist = std::numeric_limits<float>::max();
		float bestDist2 = std::numeric_limits<float>::max();
		int bestIdx = -1;
		cv::Point3f l = lines2[i];
		float lxyz_norm = std::sqrt(l.x * l.x + l.y * l.y);
		
		for (int j = 0; j < undistortedKeypoints2.size(); j++) {
			float x = undistortedKeypoints2[j].getX();
			float y = undistortedKeypoints2[j].getY();
			float disPointLine = std::abs(x * l.x + y * l.y + l.z) / lxyz_norm;
			if (disPointLine < acceptedDist) {
				float dist = cv::norm(cvDes1, cvDescriptor2.row(j), cv::NORM_L2);
				if (dist < bestDist)
				{
					bestDist2 = bestDist;
					bestDist = dist;
					bestIdx = j;
				}
				else if (dist < bestDist2)
				{
					bestDist2 = dist;
				}
			}
		}
		if ((bestIdx != -1) && (bestDist < m_matchingDistanceMax) && (bestDist < m_distanceRatio * bestDist2) && (checkMatches[bestIdx])) {
			matches.push_back(DescriptorMatch(indices1[i], bestIdx, bestDist));
			checkMatches[bestIdx] = false;
		}
	}
#endif 
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}  // end of namespace SolAR
