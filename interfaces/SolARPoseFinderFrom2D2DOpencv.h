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

#ifndef SOLARPOSEFINDERFROM2D2DOPENCV_H
#define SOLARPOSEFINDERFROM2D2DOPENCV_H


#include <vector>
#include "opencv2/core.hpp"
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
#include "SolAROpencvAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
* @class SolARPoseFinderFrom2D2DOpencv
* @brief <B>Finds the camera pose based on a 2D-2D points correspondences between two images.</B>
* <TT>UUID: 4d369049-809c-4e99-9994-5e8167bab808</TT>
*
* This component first compute the essential Matrix between the two images, and then estimate the pose of the second camera (position and orientation of the camera defined in the world coordinate system)
* knowing the pose of the first camera. To do so, the component needs to know the intrinsic parameter of the camera.
* During the essential matrix estimation, the OpenCV method filters the matches based on a RANSAC approach by removing those that are to distant to the epipolar line.
* The estimation of the pose of the second camera is based on the recoverPose method of OpenCV which decomposes the essential matrix and selects among the four solutions the one that have the most important number of 3D points in front of the camera (cheirality check).
*/


class SOLAROPENCV_EXPORT_API SolARPoseFinderFrom2D2DOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::solver::pose::I3DTransformFinderFrom2D2D
{
public:
    ///@brief SolARPoseFinderFrom2D2DOpencv constructor;
    SolARPoseFinderFrom2D2DOpencv();
    ///@brief SolARPoseFinderFrom2D2DOpencv destructor;
    ~SolARPoseFinderFrom2D2DOpencv();

    /// @brief this method is used to set intrinsic parameters and distorsion of the camera
    /// @param[in] Camera calibration matrix parameters.
    /// @param[in] Camera distorsion parameters.
    void setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) override;

    /// @brief Estimates camera pose from a set of 2D points of the first image which match with a set of 2D points of the second image.
    /// @param[in] pointsView1, Set of 2D points seen in view 1.
    /// @param[in] pointsView2, Set of 2D points seen in view 2 and matching with the 2D points of the view 1.
    /// @param[in] poseView1, Camera pose (3D transform of the camera of the view1 defined in world corrdinate system).
    /// @param[out] poseView2, Camera pose (3D transform of the camera of the view2 defined in world corrdinate system).
    /// @param[in|out] inlierMatches, a vector of matches that will be used for the pose estimation. This vector wll be updates as some input matches will be considered as outliers. If this vector is empty, we consider that the ith point of pointsView1 matches with the ith point of pointsView2.
    FrameworkReturnCode estimate(const std::vector<SRef<Point2Df>> & matchedPointsView1,
                                 const std::vector<SRef<Point2Df>> & matchedPointsView2,
                                 const Transform3Df& poseView1,
                                 Transform3Df & poseView2,
                                 std::vector<DescriptorMatch>& inlierMatches) override;

    /// @brief Estimates camera pose from a set of keypoints of the first image which match with a set of keypoints of the second image.
    /// @param[in] pointsView1, Set of keypoints seen in view 1.
    /// @param[in] pointsView2, Set of keypoints seen in view 2 and matching with the 2D points of the view 1.
    /// @param[in] poseView1, Camera pose (3D transform of the camera of the view1 defined in world corrdinate system).
    /// @param[out] poseView2, Camera pose (3D transform of the camera of the view2 defined in world corrdinate system).
    /// @param[in|out] inlierMatches, a vector of matches that will be used for the pose estimation. This vector wll be updates as some input matches will be considered as outliers. If this vector is empty, we consider that the ith point of pointsView1 matches with the ith point of pointsView2.
    FrameworkReturnCode estimate(const std::vector<SRef<Keypoint>> & matchedPointsView1,
                                 const std::vector<SRef<Keypoint>> & matchedPointsView2,
                                 const Transform3Df& poseView1,
                                 Transform3Df & poseView2,
                                 std::vector<DescriptorMatch>& inlierMatches) override;

    void unloadComponent () override final;


private:


    ///  @brief threshold to define which point are ouliers
    ///  Here we are using a RANSAC method to remove outlier.
    ///  This attribute is the ratio between the maximum distance in pixels between source points and the maximum distance in pixels to the epipolar line for which point is considered as a outlier.
    ///  The higher is this ratio, the more you will keep inliers to estimate your 2D transform, but the less this estimation will be correct.
    ///  By default, this value is set to the one proposed by [Snavely07 4.1]
    float m_outlierDistanceRatio = 0.006;

    /// @brief The probability that the algorithm produces a useful result.
    float m_confidence = 0.99f;

    CamCalibration m_camCalibration;
    CamDistortion m_camDistorsion;
};

}
}
}

#endif // SOLARPOSEFINDERFROM2D2DOPENCV_H
