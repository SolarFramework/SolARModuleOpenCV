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

#ifndef SOLARDESCRIPTORMATCHERREGIONOPENCV_H
#define SOLARDESCRIPTORMATCHERREGIONOPENCV_H

#include "api/features/IDescriptorMatcherRegion.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include <string>
#include <limits>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/calib3d.hpp>
#include "datastructure/DescriptorMatch.h"
#include "datastructure/DescriptorBuffer.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARDescriptorMatcherRegionOpencv
 * @brief <B>Matches two sets of descriptors based on region constraints.</B>
 * <TT>UUID: a12a8706-299b-4981-b12b-60717ef3b160</TT>
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

class SOLAROPENCV_EXPORT_API SolARDescriptorMatcherRegionOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::features::IDescriptorMatcherRegion {
public:
    /// @brief SolARDescriptorMatcherRegionOpencv constructor
    SolARDescriptorMatcherRegionOpencv();

    /// @brief SolARDescriptorMatcherRegionOpencv destructor
    ~SolARDescriptorMatcherRegionOpencv() override;

    /// @brief Match each descriptor input with descriptors of a frame in a region. The searching space is a circle which is defined by a 2D center and a radius
    /// @param[in] points2D The center points of searching regions
    /// @param[in] descriptors The descriptors organized in a vector of dedicated buffer structure.
    /// @param[in] frame The frame contains descriptors to match.
    /// @param[out] matches A vector of matches representing pairs of indices relatively to the first and second set of descriptors.
    /// @return FrameworkReturnCode::_SUCCESS if matching succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode match(const std::vector<SolAR::datastructure::Point2Df> & points2D,
                              const std::vector<SRef<SolAR::datastructure::DescriptorBuffer>> & descriptors,
                              const SRef<SolAR::datastructure::Frame> frame,
                              std::vector<SolAR::datastructure::DescriptorMatch> &matches,
                              const float radius = -1.f,
                              const float matchingDistanceMax = -1.f) override;

    /// @brief Match each descriptor input with descriptors of a frame in a region. The searching space is a circle which is defined by a 2D center and a radius
    /// @param[in] currentFrame the current frame.
    /// @param[in] lastFrame the last frame.
    /// @param[out] matches a vector of matches between two frames representing pairs of keypoint indices relatively.
    /// @param[in] radius the radius of search region around each keypoint of the last frame.
    /// @param[in] matchingDistanceMax the maximum distance to valid a match.
    /// @return FrameworkReturnCode::_SUCCESS if matching succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode match(const SRef<datastructure::Frame> currentFrame,
                              const SRef<datastructure::Frame> lastFrame,
                              std::vector<datastructure::DescriptorMatch> &matches,
                              const float radius = -1.f,
                              const float matchingDistanceMax = -1.f) override;

    void unloadComponent () override final;

private:
    float m_distanceRatio = 0.75f;
	float m_radius = 5.f;
	float m_matchingDistanceMax = 500.f;
    cv::FlannBasedMatcher m_matcher;

};

}
}
}  // end of namespace SolAR

#endif // SOLARDESCRIPTORMATCHERREGIONOPENCV_H
