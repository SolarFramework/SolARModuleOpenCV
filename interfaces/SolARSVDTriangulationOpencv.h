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

#include "ComponentBase.h"
#include "SolAROpencvAPI.h"

#include "api/solver/map/ITriangulator.h"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

namespace SolAR {
    using namespace datastructure;
    namespace MODULES {
        namespace OPENCV {
        /**
         * @class SolARSVDTriangulationOpencv
         * @brief Triangulates set of corresponding 2D-2D points correspondances with known respective camera poses based on opencv SVD.
         */
            class SOLAROPENCV_EXPORT_API SolARSVDTriangulationOpencv : public org::bcom::xpcf::ComponentBase,
                public api::solver::map::ITriangulator {
                    public:
                        ///@brief SolARSVDTriangulationOpencv constructor.
                        SolARSVDTriangulationOpencv();
                        ///@brief SolARSVDTriangulationOpencv destructor.
                       ~SolARSVDTriangulationOpencv();
                        /// @brief Triangulates two homogeneous 2d_points {u,v,1.0} in an iterative way based on SVD linear system solving.
                        /// @param[in] First homogeneous 2d_point.
                        /// @param[in] Second homogeneous 2d_point.
                        /// @return Triangulated homogeneous 3d_point.
                        cv::Mat_<double> iterativeLinearTriangulation(cv::Point3d &u,
                                                                      cv::Matx34d&P,
                                                                      cv::Point3d&u1,
                                                                      cv::Matx34d&P1);
                        /// @brief Triangulates two homogeneous 2d_points {u,v,1.0} based on SVD linear system solving.
                        /// @param[in] First homogeneous 2d_point.
                        /// @param[in] Second homogeneous 2d_point.
                        /// @return Triangulated homogeneous 3d_point.
                        cv::Mat_<double> linearTriangulation(cv::Point3d &u,
                                                                      cv::Matx34d&P,
                                                                      cv::Point3d&u1,
                                                                      cv::Matx34d&P1);
                        /// @brief Triangulates two sets of 2d_points seen in two different views based on SVD linear system solving.
                        /// @param[in] First set of 2d_point seen in view_1.
                        /// @param[in] Second set of 2d_point seen in view_2.
                        /// @param[in] First camera pose in the world coordinate system.
                        /// @param[in] Second camera pose in the world coordinate system.
                        /// @param[in] Camera calibration matrix parameters.
                        /// @param[in] Camera distorsion parameters.
                        /// @param[out] Set of triangulated 3d_points.
                        FrameworkReturnCode triangulate(const std::vector<SRef<Point2Df>>& pt2d_1,
                                         const std::vector<SRef<Point2Df>>& pt2d_2,
                                         const SRef<Pose>&p1,
                                         const SRef<Pose>&p2,
                                         const CamCalibration&cam,
                                         const CamDistortion&dist,
                                         std::vector<SRef<Point3Df>>& pt3d);

                        void unloadComponent () override final;

                    };

            }
    }
}




#endif // SOLARMATCHESFILTEROPENCV_H
