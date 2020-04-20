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

#include <vector>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
* @class SolARSVDTriangulationOpencv
* @brief <B>Triangulates a set of corresponding 2D-2D points correspondences with known respective camera poses based on opencv SVD.</B>
* <TT>UUID: 85274ecd-2914-4f12-96de-37c6040633a4</TT>
*
*/

class SOLAROPENCV_EXPORT_API SolARSVDTriangulationOpencv : public org::bcom::xpcf::ComponentBase,
    public api::solver::map::ITriangulator {
public:
    ///@brief SolARSVDTriangulationOpencv constructor.
    SolARSVDTriangulationOpencv();
    ///@brief SolARSVDTriangulationOpencv destructor.
   ~SolARSVDTriangulationOpencv() override;

    /// @brief this method is used to set intrinsic parameters and distorsion of the camera
    /// @param[in] Camera calibration matrix parameters.
    /// @param[in] Camera distorsion parameters.
    void setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams)  override;

    /// @brief Convert  the point cloud to opencv structure for CV processing.
    /// @param[in] Set of triangulated 3d_points.
    /// @return Set of triangulated 3d_points expressed with opencv data structure.
    double getReprojectionErrorCloud(const std::vector<CloudPoint> & original);

   /// @brief Triangulates two homogeneous 2d_points {u,v,1.0} in an iterative way based on SVD linear system solving.
    /// @param[in] First homogeneous 2d_point.
    /// @param[in] Second homogeneous 2d_point.
    /// @return Triangulated homogeneous 3d_point.
    cv::Mat_<double> iterativeLinearTriangulation(const cv::Point3d & u,
                                                  const cv::Matx34d & P,
                                                  const cv::Point3d & u1,
                                                  const cv::Matx34d & P1);

    /// @brief Triangulates two homogeneous 2d_points {u,v,1.0} based on SVD linear system solving (AX=B).
    /// @param[in] First homogeneous 2d_point.
    /// @param[in] Second homogeneous 2d_point.
    /// @return Triangulated homogeneous 3d_point.
    cv::Mat_<double> linearTriangulation(const cv::Point3d & u,
                                         const cv::Matx34d & P,
                                         const cv::Point3d & u1,
                                         const cv::Matx34d & P1);

    /// @brief Triangulates two homogeneous 2d_points {u,v,1.0} based on SVD linear system solving (AX=0).
    /// @param[in] First homogeneous 2d_point.
    /// @param[in] Second homogeneous 2d_point.
    /// @param [in,out] error Triangulation
    /// @return Triangulated homogeneous 3d_point.
    cv::Mat_<double> LinearLSTriangulation(const cv::Point2d & u,       //homogenous image point (u,v,1)
                                           const cv::Matx34d & P,       //camera 1 matrix
                                           const cv::Point2d & u1,      //homogenous image point in 2nd camera
                                           const cv::Matx34d & P1,       //camera 2 matrix
                                           double & error);

	bool lineTriangulation(	const Keyline & kl1, const Keyline & kl2,
							const cv::Mat & pose1Inv, const cv::Mat & pose2Inv,
							const cv::Mat & proj1, const cv::Mat & proj2,
							const cv::Mat & F12,
							Edge3Df & line3D, double & error);

    /// @brief triangulate pairs of points 2d captured from two views with differents poses (with respect to the camera instrinsic parameters).
    /// @param[in] pointsView1, set of 2D points seen in view_1.
    /// @param[in] pointsView2, set of 2D points seen in view_2.
    /// @param[in] matches, the matches between the keypoints of the view1 and the keypoints of the view 2.
    /// @param[in] working_views, a pair representing the id of the two views
    /// @param[in] poseView1, camera pose in the world coordinates system of the view_1 expressed as a Transform3D.
    /// @param[in] poseView2, camera pose in the world coordinates system of the view_2 expressed as a Transform3D..
    /// @param[out] pcloud, Set of triangulated 3d_points.
    /// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
    double triangulate(const std::vector<Point2Df> & pt2d_1,
                       const std::vector<Point2Df> & pt2d_2,
                       const std::vector<DescriptorMatch> & matches,
                       const std::pair<unsigned int,unsigned int> & working_views,
                       const Transform3Df & poseView1,
                       const Transform3Df & poseView2,
                       std::vector<CloudPoint> & pcloud) override;

    /// @brief triangulate pairs of points 2d captured from two views with differents poses (with respect to the camera instrinsic parameters).
    /// @param[in] pointsView1, set of keypoints seen in view_1.
    /// @param[in] pointsView2, set of keypoints seen in view_2.
    /// @param[in] matches, the matches between the keypoints of the view1 and the keypoints of the view 2.
    /// @param[in] working_views, a pair representing the id of the two views
    /// @param[in] poseView1, Camera pose in the world coordinates system of the view_1 expressed as a Transform3D.
    /// @param[in] poseView2, Camera pose in the world coordinates system of the view_2 expressed as a Transform3D..
    /// @param[out] pcloud, Set of triangulated 3d_points.
    /// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
    double triangulate(const std::vector<Keypoint> & keypointsView1,
                       const std::vector<Keypoint> & keypointsView2,
                       const std::vector<DescriptorMatch> & matches,
                       const std::pair<unsigned int,unsigned int> & working_views,
                       const Transform3Df & poseView1,
                       const Transform3Df & poseView2,
                       std::vector<CloudPoint> & pcloud) override;

	/// @brief triangulate pairs of points 2d captured from two views with differents poses (with respect to the camera instrinsic parameters).
	/// @param[in] pointsView1, set of keypoints seen in view_1.
	/// @param[in] pointsView2, set of keypoints seen in view_2.
	/// @param[in] descriptor1, set of descriptors in view_1.
	/// @param[in] descriptor2, set of descriptors in view_2.
	/// @param[in] matches, the matches between the keypoints of the view1 and the keypoints of the view 2.
	/// @param[in] working_views, a pair representing the id of the two views
	/// @param[in] poseView1, Camera pose in the world coordinates system of the view_1 expressed as a Transform3D.
	/// @param[in] poseView2, Camera pose in the world coordinates system of the view_2 expressed as a Transform3D..
	/// @param[out] pcloud, Set of triangulated 3d_points.
	/// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
	double triangulate(	const std::vector<Keypoint> & keypointsView1,
						const std::vector<Keypoint> & keypointsView2,
						const SRef<DescriptorBuffer> & descriptor1,
						const SRef<DescriptorBuffer> & descriptor2,
						const std::vector<DescriptorMatch> & matches,
						const std::pair<unsigned int, unsigned int> & working_views,
						const Transform3Df & poseView1,
						const Transform3Df & poseView2,
						std::vector<CloudPoint> & pcloud) override;

	/// @brief triangulate pairs of points 2d captured from current keyframe with its reference keyframe using their poses (with respect to the camera instrinsic parameters).
	/// @param[in] curKeyframe, current keyframe.
	/// @param[in] matches, the matches between the keypoints of the view1 and the keypoints of the view 2.
	/// @param[out] pcloud, Set of triangulated 3d_points.
	/// @return the mean re-projection error (mean distance in pixels between the original 2D points and the projection of the reconstructed 3D points)
    double triangulate(	const SRef<Keyframe> & curKeyframe,
                        const std::vector<DescriptorMatch> & matches,
                        std::vector<CloudPoint> & pcloud) override;

	/// @brief triangulate pairs of 2D keylines captured from two different views with their associated poses
	/// @param[in] keylines1, set of keylines detected in the first view.
	/// @param[in] keylines2, set of keylines detected in the second view.
	/// @param[in] matches, the matches between the keylines detected in each view.
	/// @param[in] pose1, camera pose of the first view.
	/// @param[in] pose2, camera pose of the second view.
	/// @param[out] lines3D, set of triangulated 3D lines.
	/// @param[out] indices, set of indices to recover the 2D keylines from which the 3D line was triangulated.
	/// @return the mean re-projection error
	double triangulate( const std::vector<Keyline> & keylines1,
						const std::vector<Keyline> & keylines2,
						const std::vector<DescriptorMatch> & matches,
						const Transform3Df & pose1,
						const Transform3Df & pose2,
						std::vector<Edge3Df> & lines3D,
						std::vector<int> & indices) override;

    void unloadComponent () override final;

 private:
	double distancePointLine2D(const cv::Mat & line, const cv::Mat & point);
	
	bool solvePoint3DLine(	const cv::Mat & l1, const cv::Mat & l2,
							const cv::Mat & proj1, const cv::Mat & proj2,
							const cv::Mat & point2D,
							cv::Mat & point3D,
							double & error);

    // Camera calibration matrix
    cv::Mat_<double> m_camMatrix;
    // inverse of the Camera calibration matrix
    cv::Mat_<double> m_Kinv;
    // Camera distortion parameters
    cv::Mat_<double> m_camDistorsion;

};

}
}
}




#endif // SOLARMATCHESFILTEROPENCV_H
