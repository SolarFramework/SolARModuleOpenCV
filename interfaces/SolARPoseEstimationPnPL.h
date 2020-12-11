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

#ifndef SolARPoseEstimationPnPL_H
#define SolARPoseEstimationPnPL_H

#include <opencv2/core.hpp>
#include "api/solver/pose/I3DTransformFinderFrom2D3DPointLine.h"
#include "datastructure/Image.h"
#include "datastructure/Keyline.h"
#include "SolAROpencvAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
* @class SolARPoseEstimationPnPL
* @brief <B>Finds the camera pose of 2D-3D points and lines correspondances based on the PnPL algorithm.</B>
* <TT>UUID: 19c2d03c-73d6-4f7c-a7cf-8c42bfa9b575</TT>
*
*/

class SOLAROPENCV_EXPORT_API SolARPoseEstimationPnPL : public org::bcom::xpcf::ConfigurableBase,
	public api::solver::pose::I3DTransformFinderFrom2D3DPointLine
{
public:
	/// @brief SolARPoseEstimationOPnPL contructor
	SolARPoseEstimationPnPL();
	/// @brief SolARPoseEstimationOPnPL destructor
	~SolARPoseEstimationPnPL();

	/// @brief Estimates camera pose from a set of 2D image points and 2D lines and their corresponding 3D world points and lines.
	/// @param[in] imagePoints: set of 2D points.
	/// @param[in] worldPoints: set of 3D points.
	/// @param[in] imageLines: set of 2D lines.
	/// @param[in] worldLines: set of 3D lines.
	/// @param[out] pose: camera pose (pose of the camera defined in world coordinate system) expressed as a <Transform3Df>.
	/// @param[in] initialPose: (optional) a <Transform3Df> to initialize the pose (reducing convergence time and improving success rate).
	FrameworkReturnCode estimate(	const std::vector<Point2Df> & imagePoints,
									const std::vector<Point3Df> & worldPoints,
									const std::vector<Edge2Df> & imageLines,
									const std::vector<Edge3Df> & worldLines,
									Transform3Df & pose,
									const Transform3Df & initialPose = Transform3Df::Identity()) override;

	/// @brief Estimates camera pose from a set of 2D image points and 2D lines and their corresponding 3D world points and lines,
	/// and performing RANSAC estimation iteratively to deduce inliers.
	/// @param[in] imagePoints: set of 2D points.
	/// @param[in] worldPoints: set of 3D points.
	/// @param[in] imageLines: set of 2D lines.
	/// @param[in] worldLines: set of 3D lines.
	/// @param[out] imagePoints_inliers: set of 2D points suspected as inliers by RANSAC.
	/// @param[out] worldPoints_inliers: set of 3D points suspected as inliers by RANSAC.
	/// @param[out] imageLines_inliers: set of 2D lines suspected as inliers by RANSAC.
	/// @param[out] worldLines_inliers: set of 3D lines suspected as inliers by RANSAC.
	/// @param[out] pointInliers: boolean vector to store whether a point is considered as an inlier or as an outlier.
	/// @param[out] lineInliers: boolean vector to store whether a line is considered as an inlier or as an outlier.
	/// @param[out] pose: camera pose (pose of the camera defined in world coordinate system) expressed as a <Transform3Df>.
	/// @param[in] initialPose: (optional) a <Transform3Df> to initialize the pose (reducing convergence time and improving success rate).
	FrameworkReturnCode estimateRansac(	const std::vector<Point2Df> & imagePoints,
										const std::vector<Point3Df >& worldPoints,
										const std::vector<Edge2Df> & imageLines,
										const std::vector<Edge3Df> & worldLines,
										std::vector<Point2Df>& imagePoints_inliers,
										std::vector<Point3Df>& worldPoints_inliers,
										std::vector<Edge2Df> & imageLines_inliers,
										std::vector<Edge3Df> & worldLines_inliers,
										std::vector<bool> & pointInliers,
										std::vector<bool> & lineInliers,
										Transform3Df & pose,
										const Transform3Df & initialPose = Transform3Df::Identity()) override;

	/// @brief this method is used to set intrinsic parameters and distortion of the camera
	/// @param[in] Camera calibration matrix parameters.
	/// @param[in] Camera distortion parameters.
	void setCameraParameters(const CamCalibration & intrinsicParams,
							 const CamDistortion & distortionParams) override;

	void unloadComponent() override final;

private:
	// PnPL classic approach
	bool pnpl_main(	const std::vector<Point3Df> & pt3d,
					const std::vector<Point2Df> & pt2d,
					const std::vector<Edge3Df> & ln3d,
					const std::vector<Edge2Df> & ln2d,
					cv::Mat & R0, cv::Mat & t0);

	// [WIP] OPnPL
	bool opnpl_main1(	const std::vector<Point3Df> & pt3d,
						const std::vector<Point2Df> & pt2d,
						const std::vector<Edge3Df> & ln3d,
						const std::vector<Edge2Df> & ln2d,
						cv::Mat & R0, cv::Mat & t0,
						float error);

	float getPointReprojError(const Point2Df pt2D, const Point3Df pt3D, const cv::Mat& R, const cv::Mat& t);

	float getLineReprojError(const Edge2Df ln2D, const Edge3Df ln3D, const cv::Mat& R, const cv::Mat& t);

	cv::Point3f normalizedLineCoeff(const Edge2Df line);

	float algebraicPointLineError(const Point3Df P, const cv::Point3f line2DCoeffs, const cv::Matx34f Pose);
	float algebraicLineSegmentError(const Edge3Df line3D, const Edge2Df line2D, const cv::Matx34f Pose);
	float lineSegmentError(const std::vector<Edge3Df> lines3D, const std::vector<Edge2Df> lines2D, const Transform3Df pose);

	cv::Mat m_camMatrix;
	cv::Mat m_camDistortion;
	cv::Mat m_Kinv;
};
}
}
}

#endif // SolARPoseEstimationPnPL_H