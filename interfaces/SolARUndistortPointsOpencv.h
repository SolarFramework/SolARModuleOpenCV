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

#ifndef SOLARUNDISTORTPOINTS_H
#define SOLARUNDISTORTPOINTS_H

#include "xpcf/component/ComponentBase.h"
#include "SolAROpencvAPI.h"
#include "api/geom/IUndistortPoints.h"
#include "opencv2/core.hpp"
#include <vector>

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARUndistortPointsOpencv
* @brief <B>Undistorts a set of points according to the distortion matrix of a camera.</B>
* <TT>UUID: d926e249-8b7f-46e0-8cbd-f981ceb8f921</TT>
*
*/

class SOLAROPENCV_EXPORT_API SolARUndistortPointsOpencv : public org::bcom::xpcf::ComponentBase,
    public api::geom::IUndistortPoints
{
public:
    SolARUndistortPointsOpencv();
    ~SolARUndistortPointsOpencv() = default;

    void unloadComponent () override final;

	/// @brief This method corrects undistortsion to a set of 2D points
	/// @param[in] inputPoints the set of 2D points to correct
	/// @param[out] outputPoints the  undistorted 2D Points
	/// @return FrameworkReturnCode::_SUCCESS_ if 2D transformation succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode undistort(const std::vector<datastructure::Point2Df> & inputPoints, 
								  std::vector<datastructure::Point2Df> & outputPoints) override;

	/// @brief This method corrects undistortsion to a set of 2D keypoints
	/// @param[in] inputKeypoints the set of 2D keypoints to correct
	/// @param[out] outputKeypoints the  undistorted 2D keypoints
	/// @return FrameworkReturnCode::_SUCCESS_ if 2D transformation succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode undistort(const std::vector<datastructure::Keypoint> & inputKeypoints,
								  std::vector<datastructure::Keypoint> & outputKeypoints) override;

	/// @brief this method is used to set intrinsic parameters and distorsion of the camera
	/// @param[in] Camera calibration matrix parameters.
	/// @param[in] Camera distorsion parameters.
	void setCameraParameters(const datastructure::CamCalibration & intrinsicParams,
							 const datastructure::CamDistortion & distorsionParams) override;
    

private:
    datastructure::CamCalibration m_intrinsic_parameters;
    datastructure::CamDistortion m_distortion_parameters;

    cv::Mat m_camMatrix;
    cv::Mat m_camDistortion;
};

}
}
}

#endif // SOLARUNDISTORTPOINTS_H
