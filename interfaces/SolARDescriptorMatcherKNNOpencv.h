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
#include <limits>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include "datastructure/DescriptorMatch.h"
#include "datastructure/DescriptorBuffer.h"

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
 * @SolARComponentProperty{ radius,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.5f }}
 * @SolARComponentProperty{ matchingDistanceMax,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 500.f }}
 * @SolARComponentPropertiesEnd
 * 
 * 
 */

class SOLAROPENCV_EXPORT_API SolARDescriptorMatcherKNNOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::features::IDescriptorMatcher {
public:
    SolARDescriptorMatcherKNNOpencv();
    ~SolARDescriptorMatcherKNNOpencv() override;
    void unloadComponent () override final;

    /// @brief Matches two descriptors desc1 and desc2 respectively based on KNN search strategy.
    /// [in] desc1: source descriptor.
    /// [in] desc2: target descriptor.
    /// [out] matches: ensemble of detected matches, a pair of source/target indices.
    ///@return IDescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK if succeed.
  IDescriptorMatcher::RetCode match(
            const SRef<datastructure::DescriptorBuffer> desc1,
            const SRef<datastructure::DescriptorBuffer> desc2,
            std::vector<datastructure::DescriptorMatch> & matches) override;
  /// @brief Matches a  descriptor desc1 with an ensemble of descriptors desc2 based on KNN search strategy.
  /// [in] desc1: source descriptor.
  /// [in] desc2: target descriptors.
  /// [out] matches: ensemble of detected matches, a pair of source/target indices.
  ///@return IDescriptorMatcher::RetCode::DESCRIPTORS_MATCHER_OK if succeed.
    IDescriptorMatcher::RetCode match(
           const SRef<datastructure::DescriptorBuffer> descriptors1,
           const std::vector<SRef<datastructure::DescriptorBuffer>> & descriptors2,
           std::vector<datastructure::DescriptorMatch> & matches) override;

	/// @brief Match each descriptor input with descriptors of a frame in a region. The searching space is a circle which is defined by a 2D center and a radius
	/// @param[in] points2D The center points of searching regions
	/// @param[in] descriptors The descriptors organized in a vector of dedicated buffer structure.
	/// @param[in] frame The frame contains descriptors to match.
	/// @param[out] matches A vector of matches representing pairs of indices relatively to the first and second set of descriptors.
	/// @return DesciptorMatcher::DESCRIPTORS_MATCHER_OK if matching succeeds, DesciptorMatcher::DESCRIPTORS_DONT_MATCH if the types of descriptors are different, DesciptorMatcher::DESCRIPTOR_TYPE_UNDEFINED if one of the descriptors set is unknown, or DesciptorMatcher::DESCRIPTOR_EMPTY if one of the set is empty.
	virtual IDescriptorMatcher::RetCode matchInRegion(
		const std::vector<datastructure::Point2Df> & points2D,
		const std::vector<SRef<datastructure::DescriptorBuffer>> & descriptors,
		const SRef<datastructure::Frame> frame,
		std::vector<datastructure::DescriptorMatch> &matches,
		const float radius = 0.f,
		const float matchingDistanceMax = 0.f
	) override;


private:

    /// @brief distance ratio used to keep good matches.
    /// Several matches can correspond to a given keypoint of the first image. The first match with the best score is always retained.
    /// But here, we can also retain the second match if its distance or score is greater than the score of the best match * m_distanceRatio.
    float m_distanceRatio = 0.75f;

	float m_radius = 5.f;

	float m_matchingDistanceMax = 500.f;

    int m_id;
    cv::FlannBasedMatcher m_matcher;

    IDescriptorMatcher::RetCode match(
            SRef<datastructure::DescriptorBuffer>& descriptors1,
            SRef<datastructure::DescriptorBuffer>& descriptors2,
            std::vector<std::vector< cv::DMatch >>& matches,int nbOfMatches);

};

}
}
}  // end of namespace SolAR

#endif // SOLARDESCRIPTORMATCHERKNNOPENCV_H
