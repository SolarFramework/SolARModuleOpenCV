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

#ifndef SOLARDESCRIPTORMATCHERKNNOPENCV_H
#define SOLARDESCRIPTORMATCHERKNNOPENCV_H

#include "base/features/ADescriptorMatcher.h"
#include "SolAROpencvAPI.h"
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
 * @class SolARDescriptorMatcherKNNOpencv
 * @brief <B>Matches descriptors and selects k best matches for each descriptor.</B>
 * <TT>UUID: 7823dac8-1597-41cf-bdef-59aa22f3d40a</TT>
 * 
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ distanceRatio,
 *                           distance ratio used to keep good matches.<br/>
 *                             Several matches can correspond to a given keypoint of the first image. The first match with the best score is always retained.<br>
 *                             But here\, we can also retain the second match if its distance or score is greater than the score of the best match * m_distanceRatio.,
 *                           @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], default: 0.75f }} 
 * @SolARComponentPropertiesEnd
 * 
 * 
 */

class SOLAROPENCV_EXPORT_API SolARDescriptorMatcherKNNOpencv : public base::features::ADescriptorMatcher {
public:
    /// @brief SolARDescriptorMatcherKNNOpencv constructor
    SolARDescriptorMatcherKNNOpencv();

    /// @brief SolARDescriptorMatcherKNNOpencv destructor
    ~SolARDescriptorMatcherKNNOpencv() override;    

	/// @brief Match two sets of descriptors together
	/// @param[in] descriptors1 The first set of descriptors organized in a dedicated buffer structure.
	/// @param[in] descriptors2 The second set of descriptors organized in a dedicated buffer structure.
	/// @param[out] matches A vector of matches representing pairs of indices relatively to the first and second set of descriptors.
	/// @return FrameworkReturnCode::_SUCCESS if matching succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode match(const SRef<SolAR::datastructure::DescriptorBuffer> descriptors1,
                              const SRef<SolAR::datastructure::DescriptorBuffer> descriptors2,
                              std::vector<SolAR::datastructure::DescriptorMatch> & matches) override;

	org::bcom::xpcf::XPCFErrorCode onConfigured() override final;
	void unloadComponent() override;

private:
    /// @brief distance ratio used to keep good matches.
    /// Several matches can correspond to a given keypoint of the first image. The first match with the best score is always retained.
    /// But here, we can also retain the second match if its distance or score is greater than the score of the best match * m_distanceRatio.
    float m_distanceRatio = 0.75f;
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

#endif // SOLARDESCRIPTORMATCHERKNNOPENCV_H
