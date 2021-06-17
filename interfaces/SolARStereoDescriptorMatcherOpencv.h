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

#ifndef SOLARSTEREODESCRIPTORMATCHEROPENCV_H
#define SOLARSTEREODESCRIPTORMATCHEROPENCV_H

#include "api/stereo/IStereoDescriptorMatcher.h"
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARStereoDescriptorMatcherOpencv
* @brief <B>Matches two sets of descriptors from stereo images.</B>
* <TT>UUID: a2740dbd-a17d-4a48-9f3f-3ddc38479745</TT>
*
*/

class SOLAROPENCV_EXPORT_API SolARStereoDescriptorMatcherOpencv :
	public org::bcom::xpcf::ConfigurableBase,
	public api::stereo::IStereoDescriptorMatcher
{
public:
	/// @brief SolARStereoDescriptorMatcherOpencv constructor
	SolARStereoDescriptorMatcherOpencv();

	/// @brief SolARStereoDescriptorMatcherOpencv destructor
	~SolARStereoDescriptorMatcherOpencv() override;

	/// @brief Match two sets of descriptors from stereo images.
	/// @param[in] descriptors1 Descirptors of the first image.
	/// @param[in] descriptors2 Descirptors of the second image.
	/// @param[in] keypoints1 Keypoints of the first image.
	/// @param[in] keypoints2 Keypoints of the second image.
	/// @param[in] type Stereo type (horizontal or vertical).
	/// @param[out] matches A vector of matches representing pairs of indices relatively to the first and second set of descriptors.
	/// @return FrameworkReturnCode::_SUCCESS if matching succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode match(const SRef<SolAR::datastructure::DescriptorBuffer>& descriptors1,
							const SRef<SolAR::datastructure::DescriptorBuffer>& descriptors2,
							const std::vector<SolAR::datastructure::Keypoint>& keypoints1,
							const std::vector<SolAR::datastructure::Keypoint>& keypoints2,
							SolAR::datastructure::StereoType type,
							std::vector<SolAR::datastructure::DescriptorMatch> &matches) override;

	void unloadComponent() override;

private:
	/// @brief find best match
	bool findBestMatch(const cv::Mat& query, 
					const cv::Mat& descs, 
					const std::vector<int>& candidatesId,
					int& bestIdx,
					float &minDist);

	/// @brief horizontal match
	FrameworkReturnCode horizontalMatch(const cv::Mat& descriptors1,
									const cv::Mat& descriptors2,
									const std::vector<SolAR::datastructure::Keypoint>& keypoints1,
									const std::vector<SolAR::datastructure::Keypoint>& keypoints2,
									std::vector<SolAR::datastructure::DescriptorMatch> &matches);

	/// @brief vertical match
	FrameworkReturnCode verticalMatch(const cv::Mat& descriptors1,
									const cv::Mat& descriptors2,
									const std::vector<SolAR::datastructure::Keypoint>& keypoints1,
									const std::vector<SolAR::datastructure::Keypoint>& keypoints2,
									std::vector<SolAR::datastructure::DescriptorMatch> &matches);

private:
	float m_ratioRadius = 0.007f;
	float m_matchingDistanceMax = 500.f;
};
}
}
}  // end of namespace Solar
#endif // SOLARSTEREODESCRIPTORMATCHEROPENCV_H