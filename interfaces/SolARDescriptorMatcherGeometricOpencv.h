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

#ifndef SOLARDESCRIPTORMATCHERGEOMETRICOPENCV_H
#define SOLARDESCRIPTORMATCHERGEOMETRICOPENCV_H

#include "base/features/ADescriptorMatcherGeometric.h"
#include "SolAROpencvAPI.h"
#include <limits>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/calib3d.hpp>
#ifdef WITHCUDA
#include <opencv2/cudafeatures2d.hpp>
#endif

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARDescriptorMatcherGeometricOpencv
 * @brief <B>Matches two sets of descriptors based on geometric constraints.</B>
 * <TT>UUID: 389ece8b-9e29-45ae-bd60-de1784ff0931</TT>
 * 
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ distanceRatio,
 *                           distance ratio used to keep good matches.<br/>
 *                             Several matches can correspond to a given keypoint of the first image. The first match with the best score is always retained.<br>
 *                             But here\, we can also retain the second match if its distance or score is greater than the score of the best match * m_distanceRatio.,
 *                           @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], default: 0.75f }}
 * @SolARComponentProperty{ paddingRatio,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.003f }}
 * @SolARComponentProperty{ matchingDistanceMax,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 500.f }}
 * @SolARComponentPropertiesEnd
 * 
 * 
 */

class SOLAROPENCV_EXPORT_API SolARDescriptorMatcherGeometricOpencv : public base::features::ADescriptorMatcherGeometric {
public:
    /// @brief SolARDescriptorMatcherGeometricOpencv constructor
    SolARDescriptorMatcherGeometricOpencv();

    /// @brief SolARDescriptorMatcherGeometricOpencv destructor
    ~SolARDescriptorMatcherGeometricOpencv() override;

    /// @brief Match two sets of descriptors from two frames based on epipolar constraint.
    /// @param[in] descriptors1 The first set of descriptors.
    /// @param[in] descriptors2 The second set of descriptors.
    /// @param[in] undistortedKeypoints1 The first set of undistorted keypoints.
    /// @param[in] undistortedKeypoints2 The second set of undistorted keypoints.
    /// @param[in] pose1 The first pose.
    /// @param[in] pose2 The second pose.
    /// @param[in] camParams1 The intrinsic parameters of the camera 1.
    /// @param[in] camParams2 The intrinsic parameters of the camera 2.
    /// @param[out] matches A vector of matches representing pairs of indices relatively to the first and second set of descriptors.
    /// @param[in] mask The indices of descriptors in the first frame are used for matching to the second frame. If it is empty then all will be used.
    /// @return FrameworkReturnCode::_SUCCESS if matching succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode match(const SRef<SolAR::datastructure::DescriptorBuffer> descriptors1,
                              const SRef<SolAR::datastructure::DescriptorBuffer> descriptors2,
                              const std::vector<SolAR::datastructure::Keypoint> &undistortedKeypoints1,
                              const std::vector<SolAR::datastructure::Keypoint> &undistortedKeypoints2,
                              const SolAR::datastructure::Transform3Df& pose1,
                              const SolAR::datastructure::Transform3Df& pose2,
                              const SolAR::datastructure::CameraParameters & camParams1,
                              const SolAR::datastructure::CameraParameters & camParams2,
                              std::vector<SolAR::datastructure::DescriptorMatch> & matches,
                              const std::vector<uint32_t>& mask = {}) override;

	org::bcom::xpcf::XPCFErrorCode onConfigured() override final;
	void unloadComponent() override;

private:
    float m_distanceRatio = 0.75f;
    float m_paddingRatio = 0.003f;
	float m_matchingDistanceMax = 500.f;
	/// matcher type
	std::string m_type = "BruteForce";
	/// Matcher
#ifdef WITHCUDA
	cv::Ptr<cv::cuda::DescriptorMatcher> m_matcher;
#else
	cv::Ptr<cv::DescriptorMatcher> m_matcher;
#endif  
};

}
}
}  // end of namespace SolAR

#endif // SOLARDESCRIPTORMATCHERGEOMETRICOPENCV_H
