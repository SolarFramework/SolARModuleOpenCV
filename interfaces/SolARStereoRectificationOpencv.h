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

#include "api/image/IRectification.h"
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
	public api::image::IRectification
{
public:
	/// @brief SolARStereoRectificationOpencv constructor
	SolARStereoRectificationOpencv();

	/// @brief SolARStereoRectificationOpencv destructor
	~SolARStereoRectificationOpencv() override;

	/// @brief Rectify image
	/// @param[in] image The input image
	/// @param[in] rectParams The rectification parameters of camera
	/// @param[out] rectifiedImage The rectified image	
	/// @return FrameworkReturnCode::_SUCCESS if rectifying succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode rectify(SRef<SolAR::datastructure::Image> image,
								const SolAR::datastructure::RectificationParameters& rectParams,
								SRef<SolAR::datastructure::Image>& rectifiedImage) override;

	/// @brief Rectify 2D points
	/// @param[in] points2D The input 2D points
	/// @param[in] rectParams The rectification parameters of camera
	/// @param[out] rectifiedPoints2D The rectified 2D points
	/// @return FrameworkReturnCode::_SUCCESS if rectifying succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode rectify(const std::vector<SolAR::datastructure::Point2Df>& points2D,
								const SolAR::datastructure::RectificationParameters& rectParams,
								std::vector<SolAR::datastructure::Point2Df>& rectifiedPoints2D) override;

	/// @brief Rectify 2D keypoints
	/// @param[in] keypoints The input 2D keypoints
	/// @param[in] rectParams The rectification parameters of camera
	/// @param[out] rectifiedKeypoints The rectified 2D keypoints
	/// @return FrameworkReturnCode::_SUCCESS if rectifying succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode rectify(const std::vector<SolAR::datastructure::Keypoint>& keypoints,
								const SolAR::datastructure::RectificationParameters& rectParams,
								std::vector<SolAR::datastructure::Keypoint>& rectifiedKeypoints) override;

	void unloadComponent() override;

private:
	/// @brief Rectify a 2D point
	datastructure::Point2Df rectifyPoint(const datastructure::Point2Df& pt2D, 
										 const SolAR::datastructure::RectificationParameters& rectParams);
};
}
}
}  // end of namespace Solar
#endif // SOLARSTEREORECTIFICATIONOPENCV_H