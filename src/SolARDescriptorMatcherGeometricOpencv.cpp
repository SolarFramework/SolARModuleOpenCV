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

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDescriptorMatcherGeometricOpencv)

namespace SolAR {
using namespace datastructure;
using namespace api::features;
namespace MODULES {
namespace OPENCV {

SolARDescriptorMatcherGeometricOpencv::SolARDescriptorMatcherGeometricOpencv():ConfigurableBase(xpcf::toUUID<SolARDescriptorMatcherGeometricOpencv>())
{
    declareInterface<IDescriptorMatcherGeometric>(this);
    declareProperty("distanceRatio", m_distanceRatio);
    declareProperty("paddingRatio", m_paddingRatio);
    declareProperty("matchingDistanceMax", m_matchingDistanceMax);
    LOG_DEBUG(" SolARDescriptorMatcherGeometricOpencv constructor")
}

SolARDescriptorMatcherGeometricOpencv::~SolARDescriptorMatcherGeometricOpencv()
{
    LOG_DEBUG(" SolARDescriptorMatcherGeometricOpencv destructor")
}

FrameworkReturnCode SolARDescriptorMatcherGeometricOpencv::match(
            const SRef<SolAR::datastructure::Frame> frame1,
            const SRef<SolAR::datastructure::Frame> frame2,
            const SolAR::datastructure::Transform3Df& pose1,
            const SolAR::datastructure::Transform3Df& pose2,
            const SolAR::datastructure::CamCalibration& intrinsicParameters,
            std::vector<SolAR::datastructure::DescriptorMatch> & matches,
            const std::vector<uint32_t>& mask)
{
    matches.clear();
    // check if the descriptors type match
    if((frame1->getDescriptors()->getDescriptorType() != frame2->getDescriptors()->getDescriptorType()) ||
		(frame1->getDescriptors()->getNbDescriptors() == 0) || (frame2->getDescriptors()->getNbDescriptors() == 0)){
        return FrameworkReturnCode::_ERROR_;
    }

    // calculate fundametal matrix
    Transform3Df pose12 = pose1.inverse() * pose2;
    cv::Mat R12 = (cv::Mat_<float>(3, 3) << pose12(0, 0), pose12(0, 1), pose12(0, 2),
        pose12(1, 0), pose12(1, 1), pose12(1, 2),
        pose12(2, 0), pose12(2, 1), pose12(2, 2));
    cv::Mat T12 = (cv::Mat_<float>(3, 1) << pose12(0, 3), pose12(1, 3), pose12(2, 3));
    cv::Mat T12x = (cv::Mat_<float>(3, 3) << 0, -T12.at<float>(2), T12.at<float>(1),
        T12.at<float>(2), 0, -T12.at<float>(0),
        -T12.at<float>(1), T12.at<float>(0), 0);
    cv::Mat K(3, 3, CV_32FC1, (void *)intrinsicParameters.data());
    cv::Mat F = K.t().inv() * T12x * R12 * K.inv();

    // get input points in the first frame
    const std::vector<Keypoint>& keypointsUn1 = frame1->getUndistortedKeypoints();
    std::vector<cv::Point2f> pts1;
	std::vector<uint32_t> indices1;
    if (mask.size() == 0){
		for (int i = 0; i < keypointsUn1.size(); ++i) {
			pts1.push_back(cv::Point2f(keypointsUn1[i].getX(), keypointsUn1[i].getY()));
			indices1.push_back(i);
		}
    }
    else {
		indices1 = mask;
        for (int i = 0; i < mask.size(); ++i)
            pts1.push_back(cv::Point2f(keypointsUn1[mask[i]].getX(), keypointsUn1[mask[i]].getY()));
    }

    // compute epipolar lines
    std::vector<cv::Point3f> lines2;
    cv::computeCorrespondEpilines(pts1, 2, F, lines2);

    // convert frame descriptor to opencv's descriptor
	const SRef<DescriptorBuffer>& descriptors1 = frame1->getDescriptors();
	const SRef<DescriptorBuffer>& descriptors2 = frame2->getDescriptors();
    uint32_t type_conversion = SolAR::MODULES::OPENCV::SolAROpenCVHelper::deduceOpenDescriptorCVType(descriptors1->getDescriptorDataType());
    cv::Mat cvDescriptor1(descriptors1->getNbDescriptors(), descriptors1->getNbElements(), type_conversion);
    cvDescriptor1.data = (uchar*)descriptors1->data();
    cv::Mat cvDescriptor2(descriptors2->getNbDescriptors(), descriptors2->getNbElements(), type_conversion);
    cvDescriptor2.data = (uchar*)descriptors2->data();

	// match
	const std::vector<Keypoint>& keypointsUn2 = frame2->getUndistortedKeypoints();
    std::vector<bool> checkMatches(keypointsUn2.size(), true);
    float acceptedDist = m_paddingRatio * frame1->getView()->getWidth();
    for (int i = 0; i < indices1.size(); i++) {
        const cv::Mat cvDes1 = cvDescriptor1.row(indices1[i]);
        float bestDist = std::numeric_limits<float>::max();
        float bestDist2 = std::numeric_limits<float>::max();
        int bestIdx = -1;
        for (int j = 0; j < keypointsUn2.size(); j++) {
			float x = keypointsUn2[j].getX();
			float y = keypointsUn2[j].getY();
			cv::Point3f l = lines2[i];
			float disPointLine = std::abs(x * l.x + y * l.y + l.z) / std::sqrt(l.x * l.x + l.y * l.y);
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
    return FrameworkReturnCode::_SUCCESS;
}

}
}
}  // end of namespace SolAR
