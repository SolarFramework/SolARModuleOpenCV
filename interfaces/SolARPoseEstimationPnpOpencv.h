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
#include "opencv2/core.hpp"
#include "api/solver/pose/I3DTransformFinder.h"
#include "SolAROpencvAPI.h"
#include "ComponentBase.h"

namespace SolAR {
    using namespace datastructure;
    namespace MODULES {
        namespace OPENCV {
        /**
         * @class SolARPoseEstimationPnpOpencv
         * @brief Finds the camera pose of 2D-3D points correspondaces based on opencv pnp algorithm.
         */
            class SOLAROPENCV_EXPORT_API SolARPoseEstimationPnpOpencv : public org::bcom::xpcf::ComponentBase,
                public api::solver::pose::I3DTransformFinder
            {
            public:
                ///@brief SolARPoseEstimationPnpOpencv constructor;
                SolARPoseEstimationPnpOpencv();
                ///@brief SolARPoseEstimationPnpOpencv destructor;
                ~SolARPoseEstimationPnpOpencv();
                /// @brief Estimates camera pose from a set of 2D image points of their corresponding 3D  world points. The estimation is based on opencv Perspective from N Points algorithm
                /// @param[in] Set of 2d_points seen in view_1.
                /// @param[in] Set of 3d_points corresponding to view_1.
                /// @param[out] Camera pose in the world coordinates system of the view_1.
                FrameworkReturnCode estimate(const std::vector<SRef<Point2Df>> & imagePoints,
                                         const std::vector<SRef<Point3Df>> & worldPoints,
                                         Transform3Df & pose) override;
                /// @brief this method is used to set intrinsic parameters and distorsion of the camera
                /// @param[in] Camera calibration matrix parameters.
                /// @param[in] Camera distorsion parameters.
                void setCameraParameters(const CamCalibration & intrinsicParams,
                                         const CamDistortion & distorsionParams)  override;

                void unloadComponent () override final;

            private:
                cv::Mat m_camMatrix;
                cv::Mat m_camDistorsion;
            };
}
}
}

#endif // SOLARPOSEESTIMATIONPNPOPENCV_H
