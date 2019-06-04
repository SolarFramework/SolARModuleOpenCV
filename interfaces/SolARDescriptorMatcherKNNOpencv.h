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

#include "api/features/IDescriptorMatcher.h"

// Definition of SolARDescriptorMatcherOpencv Class //
// part of SolAR namespace //

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include "datastructure/DescriptorMatch.h"
#include "datastructure/DescriptorBuffer.h"

namespace SolAR {
using namespace datastructure;
using namespace api::features;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARDescriptorMatcherKNNOpencv
 * @brief <B>Matches descriptors and selects k best matches for each descriptor.</B>
 * <TT>UUID: 7823dac8-1597-41cf-bdef-59aa22f3d40a</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARDescriptorMatcherKNNOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::features::IDescriptorMatcher {
public:
    SolARDescriptorMatcherKNNOpencv();
    ~SolARDescriptorMatcherKNNOpencv();
    void unloadComponent () override final;

    /// @brief Matches two descriptors desc1 and desc2 respectively based on KNN search strategy.
    /// [in] desc1: source descriptor.
    /// [in] desc2: target descriptor.
    /// [out] matches: ensemble of detected matches, a pair of source/target indices.
    ///@return DescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK if succeed.
  DescriptorMatcher::RetCode match(
            SRef<DescriptorBuffer> desc1,
            SRef<DescriptorBuffer> desc2,
            std::vector<DescriptorMatch>& matches);
  /// @brief Matches a  descriptor desc1 with an ensemble of descriptors desc2 based on KNN search strategy.
  /// [in] desc1: source descriptor.
  /// [in] desc2: target descriptors.
  /// [out] matches: ensemble of detected matches, a pair of source/target indices.
  ///@return DescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK if succeed.
    DescriptorMatcher::RetCode match(
           SRef<DescriptorBuffer> descriptors1,
           std::vector<SRef<DescriptorBuffer>>& descriptors2,
           std::vector<DescriptorMatch>& matches
        );


private:

    /// @brief distance ratio used to keep good matches.
    /// Several matches can correspond to a given keypoint of the first image. The first match with the best score is always retained.
    /// But here, we can also retain the second match if its distance or score is greater than the score of the best match * m_distanceRatio.
    float m_distanceRatio = 0.75f;


    int m_id;
    cv::FlannBasedMatcher m_matcher;

    DescriptorMatcher::RetCode match(
            SRef<DescriptorBuffer>& descriptors1,
            SRef<DescriptorBuffer>& descriptors2,
            std::vector<std::vector< cv::DMatch >>& matches,int nbOfMatches);

};

}
}
}  // end of namespace SolAR

#endif // SOLARDESCRIPTORMATCHERKNNOPENCV_H
