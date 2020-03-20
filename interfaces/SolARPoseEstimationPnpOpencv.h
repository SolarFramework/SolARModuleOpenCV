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

#ifndef SOLARPOSEESTIMATIONPNPOPENCV_H
#define SOLARPOSEESTIMATIONPNPOPENCV_H
#include <vector>
#include <string>
#include "opencv2/core.hpp"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "datastructure/Image.h"
#include "SolAROpencvAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
* @class SolARPoseEstimationPnpOpencv
* @brief <B>Finds the camera pose of 2D-3D points correspondences based on opencv Perspective-n-Points algorithm.</B>
* <TT>UUID: 0753ade1-7932-4e29-a71c-66155e309a53</TT>
*
*/

class SOLAROPENCV_EXPORT_API SolARPoseEstimationPnpOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::solver::pose::I3DTransformFinderFrom2D3D
{
public:
    ///@brief SolARPoseEstimationPnpOpencv constructor;
    SolARPoseEstimationPnpOpencv();
    ///@brief SolARPoseEstimationPnpOpencv destructor;
    ~SolARPoseEstimationPnpOpencv();

    /// @brief Estimates camera pose from a set of 2D image points of their corresponding 3D  world points.
    /// @param[in] imagePoints, set of 2d_points seen in view_1.
    /// @param[in]  worldPoints, set of 3d_points corresponding to view_1.
    /// @param[out] pose, camera pose (pose the camera defined in world corrdinate system) expressed as a Transform3D.
    /// @param[in] initialPose (Optional), a tranfsform3D to initialize the pose (reducing the convergence time and improving its success). If your world points are planar, do not use this argument.
    FrameworkReturnCode estimate(const std::vector<Point2Df> & imagePoints,
                             const std::vector<Point3Df> & worldPoints,
                             Transform3Df & pose,
                             const Transform3Df initialPose = Transform3Df::Identity()) override;

    /// @brief this method is used to set intrinsic parameters and distorsion of the camera
    /// @param[in] Camera calibration matrix parameters.
    /// @param[in] Camera distorsion parameters.
    void setCameraParameters(const CamCalibration & intrinsicParams,
                             const CamDistortion & distorsionParams)  override;

    void unloadComponent () override final;


private:
    /// @brief Number of iterations
    int m_iterationsCount = 1000;

    /// @brief Inlier threshold value used by the RANSAC procedure. The parameter value is the maximum allowed distance between the observed and computed point projections to consider it an inlier.
    float m_reprojError = 4.0;

    /// @brief The probability that the algorithm produces a useful result.
    float m_confidence = 0.99f;

    /// @brief The minimum of number of inliers to valid a good pose estimation
    int m_NbInliersToValidPose = 10;

    /// @brief The method for solving the PnP problem (ITERATIVE, P3P, AP3P, EPNP, DLS, UPNP, IPPE, IPPE_SQUARE)
    std::string m_method = "ITERATIVE";

    cv::Mat m_camMatrix;
    cv::Mat m_camDistorsion;
};

}
}
}

#endif // SOLARPOSEESTIMATIONPNPOPENCV_H
