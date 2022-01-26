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

#ifndef SOLARDESCRIPTORMATCHERRADIUSOPENCV_H
#define SOLARDESCRIPTORMATCHERRADIUSOPENCV_H

#include "base/features/ADescriptorMatcher.h"
#include "SolAROpencvAPI.h"
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARDescriptorMatcherRadiusOpencv
 * @brief <B>Matches descriptors and selects all matches not farther than a specified distance.</B>
 * <TT>UUID: 549f7873-96e4-4eae-b4a0-ae8d80664ce5</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ maxDistance,
 *                           Threshold for the distance between matched descriptors.<br>
 *                             Distance means here metric distance (e.g. Hamming distance)\, not the distance between coordinates (which is measured in Pixels),
 *                           @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], default: 1.f }}
 * @SolARComponentPropertiesEnd
 * 
 */

class SOLAROPENCV_EXPORT_API SolARDescriptorMatcherRadiusOpencv : public base::features::ADescriptorMatcher {
public:
	SolARDescriptorMatcherRadiusOpencv();
	~SolARDescriptorMatcherRadiusOpencv();

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
    /// @brief Threshold for the distance between matched descriptors. Distance means here metric distance (e.g. Hamming distance), not the distance between coordinates (which is measured in Pixels)
    float m_maxDistance = 1.0f;
	std::string m_type = "BruteForce";
	cv::Ptr<cv::DescriptorMatcher> m_matcher;
};

}
}
}  // end of namespace SolAR

#endif // SOLARDESCRIPTORMATCHERRADIUSOPENCV_H
