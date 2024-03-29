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
namespace MODULES {
namespace OPENCV {

/**
* @class SolARPoseEstimationPnpOpencv
* @brief <B>Finds the camera pose of 2D-3D points correspondences based on opencv Perspective-n-Points algorithm.</B>
* <TT>UUID: 0753ade1-7932-4e29-a71c-66155e309a53</TT>
*
* @SolARComponentPropertiesBegin
* @SolARComponentProperty{ iterationsCount,
*                          number of iterations,
*                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 1000 }}
* @SolARComponentProperty{ reprojError,
*                          inlier threshold value used by the RANSAC procedure.<br>
*                            The parameter value is the maximum allowed distance between the observed and computed point projections to consider it an inlier,
*                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 4f}}
* @SolARComponentProperty{ confidence,
*                          the probability that the algorithm produces a useful result,
*                          @SolARComponentPropertyDescNum{ float, [0..1], 0.99f }}
* @SolARComponentProperty{ minNbInliers,
*                          the minimum of number of inliers to valid a good pose estimation,
*                          @SolARComponentPropertyDescNum{ int, [0..1MAX INT], 10 }}
* @SolARComponentProperty{ method,
*                          The method for solving the PnP problem (ITERATIVE\, P3P\, AP3P\, EPNP\, DLS\, UPNP\, IPPE\, IPPE_SQUARE),
*                          @SolARComponentPropertyDescString{ "ITERATIVE" }}
* @SolARComponentPropertiesEnd
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

    /// @brief Estimates camera pose from a set of 2D image points of their corresponding 3D world points.
    /// @param[in] imagePoints the set of 2D image points.
    /// @param[in]  worldPoints the set of 3d world points.
    /// @param[in] camParams the camera parameters.
    /// @param[out] pose camera pose (pose of the camera defined in world corrdinate system) expressed as a Transform3D.
    /// @param[in] initialPose (Optional) a transform3D to initialize the pose (reducing the convergence time and improving its success).
    /// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode estimate(const std::vector<SolAR::datastructure::Point2Df> & imagePoints,
                                 const std::vector<SolAR::datastructure::Point3Df> & worldPoints,
                                 const SolAR::datastructure::CameraParameters & camParams,
                                 SolAR::datastructure::Transform3Df & pose,
                                 const SolAR::datastructure::Transform3Df initialPose = SolAR::datastructure::Transform3Df::Identity()) override;

    void unloadComponent () override final;

private:
    void setCameraParameters(const SolAR::datastructure::CameraParameters & camParams);

private:

    /// @brief The method for solving the PnP problem (ITERATIVE, P3P, AP3P, EPNP, DLS, UPNP, IPPE, IPPE_SQUARE)
    std::string m_method = "ITERATIVE";

    cv::Mat m_camMatrix;
    cv::Mat m_camDistorsion;
};

}
}
}

#endif // SOLARPOSEESTIMATIONPNPOPENCV_H
