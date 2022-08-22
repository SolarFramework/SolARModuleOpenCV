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

#ifndef SOLARPOSEESTIMATIONPLANARPOINTSOPENCV_H
#define SOLARPOSEESTIMATIONPLANARPOINTSOPENCV_H
#include <vector>
#include "opencv2/core.hpp"
#include "api/solver/pose/I3DTransformSACFinderFrom2D3D.h"
#include "datastructure/Image.h"
#include "SolAROpencvAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARPoseEstimationPlanarPointsOpencv
 * @brief <B>Finds the camera pose of 2D-3D planar points correspondences based on opencv homography.</B>
 * <TT>UUID: 9fbadf80-251f-4160-94f8-a64dc3d40a2f</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ minNbInliers,
 *                          minimal number of inliers to validate the pose,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 10 }}
 * @SolARComponentProperty{ reprojErrorThreshold,
 *                          Inlier threshold value used by the RANSAC procedure. The parameter value is the maximum
 *                            allowed distance between the observed and computed point projections to consider it an inlier.,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.1f }}
 * @SolARComponentPropertiesEnd
 * 
 */

class SOLAROPENCV_EXPORT_API SolARPoseEstimationPlanarPointsOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::solver::pose::I3DTransformSACFinderFrom2D3D
{
public:
    ///@brief SolARPoseEstimationPlanarPointsOpencv constructor;
    SolARPoseEstimationPlanarPointsOpencv();
    ///@brief SolARPoseEstimationPlanarPointsOpencv destructor;
    ~SolARPoseEstimationPlanarPointsOpencv() override;

    /// @brief Estimates camera pose from a set of 2D image points of their corresponding 3D  world points.
    /// @param[in] inputPoints the set of 3D cloud points to project
    /// @param[in] pose the 3D pose of the camera (a 4x4 float matrix)
    /// @param[in] camParams the camera parameters.
    /// @param[out] inliers indices of inlier correspondences.
    /// @param[out] pose camera pose (pose of the camera defined in world corrdinate system) expressed as a Transform3D.
    /// @param[in] initialPose (Optional) a transform3D to initialize the pose (reducing the convergence time and improving its success).
    /// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode estimate(const std::vector<SolAR::datastructure::Point2Df> & imagePoints,
                                 const std::vector<SolAR::datastructure::Point3Df> & worldPoints,
                                 const SolAR::datastructure::CameraParameters & camParams,
                                 std::vector<uint32_t> & inliers,
                                 SolAR::datastructure::Transform3Df & pose,
                                 const SolAR::datastructure::Transform3Df initialPose = SolAR::datastructure::Transform3Df::Identity()) override;

    void unloadComponent () override final;


private:

    // minimal number of inliers to validate the pose
    int m_minNbInliers = 10;

    /// @brief Inlier threshold value used by the RANSAC procedure. The parameter value is the maximum allowed distance between the observed and computed point projections to consider it an inlier.
    float m_reprojErrorThreshold = 0.1f;


    cv::Mat m_camMatrix;
    cv::Mat m_camDistorsion;
};

}
}
}

#endif // SOLARPOSEESTIMATIONPLANARPOINTSOPENCV_H
