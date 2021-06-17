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

#ifndef SOLARSTEREORECTIFICATIONOPENCV_H
#define SOLARSTEREORECTIFICATIONOPENCV_H

#include "api/stereo/IStereoRectification.h"
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
* @class SolARStereoRectificationOpencv
* @brief <B>Rectiy image or 2D points.</B>
* <TT>UUID: bf4c7011-b7e6-453d-a755-884dac18d3ee</TT>
*
*/

class SOLAROPENCV_EXPORT_API SolARStereoRectificationOpencv :
	public org::bcom::xpcf::ConfigurableBase,
	public api::stereo::IStereoRectification
{
public:
	/// @brief SolARStereoRectificationOpencv constructor
	SolARStereoRectificationOpencv();

	/// @brief SolARStereoRectificationOpencv destructor
	~SolARStereoRectificationOpencv() override;

	/// @brief Rectify image
	/// @param[in] image The input image
	/// @param[out] rectifiedImage The rectified image
	/// @param[in] indexCamera The index of camera
	void rectify(SRef<SolAR::datastructure::Image> image,
				SRef<SolAR::datastructure::Image>& rectifiedImage,
				int indexCamera) override;

	/// @brief Rectify 2D points
	/// @param[in] points2D The input 2D points
	/// @param[out] rectifiedPoints2D The rectified 2D points
	/// @param[in] indexCamera The index of camera
	void rectify(const std::vector<SolAR::datastructure::Point2Df>& points2D,
				std::vector<SolAR::datastructure::Point2Df>& rectifiedPoints2D,
				int indexCamera) override;

	/// @brief Rectify 2D keypoints
	/// @param[in] keypoints The input 2D keypoints
	/// @param[out] rectifiedKeypoints The rectified 2D keypoints
	/// @param[in] indexCamera The index of camera
	void rectify(const std::vector<SolAR::datastructure::Keypoint>& keypoints,
				std::vector<SolAR::datastructure::Keypoint>& rectifiedKeypoints,
				int indexCamera) override;

	/// @brief Get stereo camera type
	/// @return stereo type
	SolAR::datastructure::StereoType getType() override;

	/// @brief Get baseline distance
	/// @return baseline distance
	float getBaseline() override;

	/// @brief Get rectification parameters
	/// @param[in] indexCamera Index of camera
	/// @return rectification parameters
	SolAR::datastructure::RectificationParameters getRectificationParamters(int indexCamera) override;

	void unloadComponent() override;

	org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

private:
	/// @brief Rectify a 2D point
	datastructure::Point2Df rectifyPoint(const datastructure::Point2Df& pt2D, int indexCamera);

private:
	std::string m_rectificationFile;
	cv::Size m_imageSize;
	std::vector<cv::Mat> m_R, m_P, m_intrinsic, m_distortion;
	std::vector<datastructure::RectificationParameters> m_rectificationParams;
	float m_baseline;
	datastructure::StereoType m_stereoType;
};
}
}
}  // end of namespace Solar
#endif // SOLARSTEREORECTIFICATIONOPENCV_H