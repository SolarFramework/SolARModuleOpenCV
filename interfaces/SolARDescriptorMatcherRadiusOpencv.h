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

#include "api/features/IDescriptorMatcher.h"

// Definition of SolARDescriptorMatcherRadiusOpencv Class //
// part of SolAR namespace //

#include "xpcf/component/ConfigurableBase.h"
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

class SOLAROPENCV_EXPORT_API SolARDescriptorMatcherRadiusOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::features::IDescriptorMatcher {
public:
    SolARDescriptorMatcherRadiusOpencv();
    ~SolARDescriptorMatcherRadiusOpencv();
    void unloadComponent () override final;

    /// @brief Matches two descriptors desc1 and desc2 respectively based on radius search strategy.
    /// [in] desc1: source descriptor.
    /// [in] desc2: target descriptor.
    /// [out] matches: ensemble of detected matches, a pair of source/target indices.
    ///@return IDescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK if succeed.
  IDescriptorMatcher::RetCode match(
            const SRef<datastructure::DescriptorBuffer> desc1,
            const SRef<datastructure::DescriptorBuffer> desc2,
            std::vector<datastructure::DescriptorMatch> & matches) override;

  /// @brief Matches a  descriptor desc1 with an ensemble of descriptors desc2 based on radius search strategy.
  /// [in] desc1: source descriptor.
  /// [in] desc2: target descriptors.
  /// [out] matches: ensemble of detected matches, a pair of source/target indices.
  ///@return IDescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK if succeed.
    IDescriptorMatcher::RetCode match(
           const SRef<datastructure::DescriptorBuffer> descriptors1,
           const std::vector<SRef<datastructure::DescriptorBuffer>> & descriptors2,
           std::vector<datastructure::DescriptorMatch> & matches) override;


private:
    /// @brief Threshold for the distance between matched descriptors. Distance means here metric distance (e.g. Hamming distance), not the distance between coordinates (which is measured in Pixels)
    float m_maxDistance = 1.0f;


    cv::FlannBasedMatcher m_matcher;

};

}
}
}  // end of namespace SolAR

#endif // SOLARDESCRIPTORMATCHERRADIUSOPENCV_H
