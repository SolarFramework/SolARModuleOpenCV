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


#ifndef SOLARSVDTRIANGULATIONOPENCV_H
#define SOLARSVDTRIANGULATIONOPENCV_H

#include "xpcf/component/ComponentBase.h"
#include "SolAROpencvAPI.h"

#include "api/solver/map/ITriangulator.h"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "datastructure/DescriptorBuffer.h"
#include "api/geom/IProject.h"

#include <vector>

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARSVDTriangulationOpencv
* @brief <B>Triangulates a set of corresponding 2D-2D points correspondences with known respective camera poses based on opencv SVD.</B>
* <TT>UUID: 85274ecd-2914-4f12-96de-37c6040633a4</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::geom::IProject}
* @SolARComponentInjectablesEnd
* 
*/

class SOLAROPENCV_EXPORT_API SolARSVDTriangulationOpencv : public org::bcom::xpcf::ComponentBase,
    public api::solver::map::ITriangulator {
public:
    ///@brief SolARSVDTriangulationOpencv constructor.
    SolARSVDTriangulationOpencv();

    ///@brief SolARSVDTriangulationOpencv destructor.
    ~SolARSVDTriangulationOpencv() override;

    /// @brief triangulate pairs of points 2d captured from two views with differents poses (with respect to the camera instrinsic parameters).
    /// @param[in] pointsView1 set of 2D points seen in view_1.
    /// @param[in] pointsView2 set of 2D points seen in view_2.
    /// @param[in] matches the matches between the keypoints of the view1 and the keypoints of the view 2.
    /// @param[in] working_views a pair representing the id of the two views
    /// @param[in] poseView1 camera pose in the world coordinates system of the view_1 expressed as a Transform3D.
    /// @param[in] poseView2 camera pose in the world coordinates system of the view_2 expressed as a Transform3D.
    /// @param[in] camParams1 the parameters of the camera 1.
    /// @param[in] camParams1 the parameters of the camera 2.
    /// @param[out] pcloud set of triangulated 3d_points.
    /// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
    double triangulate(const std::vector<SolAR::datastructure::Point2Df> & pointsView1,
                       const std::vector<SolAR::datastructure::Point2Df> & pointsView2,
                       const std::vector<SolAR::datastructure::DescriptorMatch> & matches,
                       const std::pair<uint32_t, uint32_t> & working_views,
                       const SolAR::datastructure::Transform3Df & poseView1,
                       const SolAR::datastructure::Transform3Df & poseView2,
                       const SolAR::datastructure::CameraParameters & camParams1,
                       const SolAR::datastructure::CameraParameters & camParams2,
                       std::vector<SRef<SolAR::datastructure::CloudPoint>> & pcloud) override;

    /// @brief triangulate pairs of points 2d captured from two views with differents poses (with respect to the camera instrinsic parameters).
    /// @param[in] keypointsView1 set of keypoints seen in view_1.
    /// @param[in] keypointsView2 set of keypoints seen in view_2.
    /// @param[in] matches the matches between the keypoints of the view1 and the keypoints of the view 2.
    /// @param[in] working_views a pair representing the id of the two views
    /// @param[in] poseView1 camera pose in the world coordinates system of the view_1 expressed as a Transform3D.
    /// @param[in] poseView2 camera pose in the world coordinates system of the view_2 expressed as a Transform3D.
    /// @param[in] camParams1 the parameters of the camera 1.
    /// @param[in] camParams1 the parameters of the camera 2.
    /// @param[out] pcloud set of triangulated 3d_points.
    /// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
    double triangulate(const std::vector<SolAR::datastructure::Keypoint> & keypointsView1,
                       const std::vector<SolAR::datastructure::Keypoint> & keypointsView2,
                       const std::vector<SolAR::datastructure::DescriptorMatch> &matches,
                       const std::pair<uint32_t, uint32_t> & working_views,
                       const SolAR::datastructure::Transform3Df & poseView1,
                       const SolAR::datastructure::Transform3Df & poseView2,
                       const SolAR::datastructure::CameraParameters & camParams1,
                       const SolAR::datastructure::CameraParameters & camParams2,
                       std::vector<SRef<SolAR::datastructure::CloudPoint>> & pcloud) override;

    /// @brief triangulate pairs of points 2d captured from two views with differents poses (with respect to the camera instrinsic parameters).
    /// @param[in] pointsView1 set of keypoints seen in view_1.
    /// @param[in] pointsView2 set of keypoints seen in view_2.
    /// @param[in] descriptor1 set of descriptors in view_1.
    /// @param[in] descriptor2 set of descriptors in view_2.
    /// @param[in] matches the matches between the keypoints of the view1 and the keypoints of the view 2.
    /// @param[in] working_views a pair representing the id of the two views
    /// @param[in] poseView1 Camera pose in the world coordinates system of the view_1 expressed as a Transform3D.
    /// @param[in] poseView2 Camera pose in the world coordinates system of the view_2 expressed as a Transform3D.
    /// @param[in] camParams1 the parameters of the camera 1.
    /// @param[in] camParams1 the parameters of the camera 2.
    /// @param[out] pcloud Set of triangulated 3d_points.
    /// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
    double triangulate(const std::vector<SolAR::datastructure::Keypoint> & keypointsView1,
                       const std::vector<SolAR::datastructure::Keypoint> & keypointsView2,
                       const SRef<SolAR::datastructure::DescriptorBuffer> & descriptor1,
                       const SRef<SolAR::datastructure::DescriptorBuffer> & descriptor2,
                       const std::vector<SolAR::datastructure::DescriptorMatch> & matches,
                       const std::pair<uint32_t, uint32_t> & working_views,
                       const SolAR::datastructure::Transform3Df & poseView1,
                       const SolAR::datastructure::Transform3Df & poseView2,
                       const SolAR::datastructure::CameraParameters & camParams1,
                       const SolAR::datastructure::CameraParameters & camParams2,
                       std::vector<SRef<SolAR::datastructure::CloudPoint>> & pcloud) override;

    /// @brief calculating 3D cloud points by triangulating pairs of matched features or using depth information of keypoints.
    /// @param[in] frame1 the first frame.
    /// @param[in] frame2 the second frame.
    /// @param[in] matches the matches between these two frames.
    /// @param[in] working_views a pair representing the id of the two views
    /// @param[out] pcloud Set of triangulated 3d_points.
    /// @param[in] onlyDepth if it is true, using only depth information of keypoints for computing 3D cloud points.
    /// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
    double triangulate(SRef<SolAR::datastructure::Frame> frame1,
                       SRef<SolAR::datastructure::Frame> frame2,
                       const std::vector<SolAR::datastructure::DescriptorMatch> &matches,
                       const std::pair<uint32_t, uint32_t> & working_views,
                       const SolAR::datastructure::CameraParameters & camParams1,
                       const SolAR::datastructure::CameraParameters & camParams2,
                       std::vector<SRef<SolAR::datastructure::CloudPoint>> & pcloud,
                       const bool& onlyDepth = false) override;

    void unloadComponent () override final;

private:
	/// @brief Triangulates two homogeneous 2d_points {u,v,1.0} in an iterative way based on SVD linear system solving.
    cv::Mat iterativeLinearTriangulation(const cv::Point3f & u1,
                                         const cv::Mat & P1,
                                         const cv::Point3f & u2,
                                         const cv::Mat & P2);

	/// @brief Triangulates two homogeneous 2d_points {u,v,1.0} based on SVD linear system solving (AX=0) from "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997.
	cv::Mat linearTriangulation(const cv::Point3f & u1,
								const cv::Mat & P1,
								const cv::Point3f & u2,
								const cv::Mat & P2);

	/// @brief Define a 3D point by unprojecting a keypoint
	datastructure::Point3Df unproject3DPoint(const datastructure::Keypoint& kp,
											 const datastructure::CameraParameters & camParams,
                                             const datastructure::Transform3Df& pose);

private:
	SRef<api::geom::IProject> m_projector;
};

}
}
}




#endif // SOLARMATCHESFILTEROPENCV_H
