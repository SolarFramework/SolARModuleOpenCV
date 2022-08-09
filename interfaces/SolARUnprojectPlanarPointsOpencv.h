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

#ifndef SOLARUNPROJECTPLANARPOINTSOPENCV_H
#define SOLARUNPROJECTPLANARPOINTSOPENCV_H
#include <vector>
#include "opencv2/core.hpp"
#include "api/geom/IUnproject.h"
#include "datastructure/Image.h"
#include "SolAROpencvAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARUnprojectPlanarPointsOpencv
* @brief <B>Recovers 3D points defined in world coordinate system from a set of 2D points defined in the image coordinate system.</B>
* <TT>UUID: 9938354d-6476-437e-8325-97e82666a46e</TT>
*
*/

class SOLAROPENCV_EXPORT_API SolARUnprojectPlanarPointsOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::geom::IUnproject
{
public:
    ///@brief SolARUnprojectPlanarPointsOpencv constructor;
    SolARUnprojectPlanarPointsOpencv();

    ///@brief SolARUnprojectPlanarPointsOpencv destructor;
    ~SolARUnprojectPlanarPointsOpencv() override;

    /// @brief This method unproject a set of 2D image points in the 3D world coordinate system
    /// @param[in] imagePoints the set of 2D points to unproject
    /// @param[in] pose the 3D pose of the camera (a 4x4 float matrix)
    /// @param[in] camParams the camera parameters
    /// @param[out] worldPoints a set of world 3D points resulting from the unprojection of the 2D image points
    /// @return FrameworkReturnCode::_SUCCESS_ if 3D projection succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode unproject(const std::vector<SolAR::datastructure::Point2Df> & imagePoints,
                                  const SolAR::datastructure::Transform3Df & pose,
                                  const SolAR::datastructure::CameraParameters & camParams,
                                  std::vector<SolAR::datastructure::Point3Df> & worldPoints) override;

    /// @brief This method unproject a set of 2D image points in the 3D world coordinate system
    /// @param[in] imageKeypoints the set of 2D keypoints to unproject
    /// @param[in] pose the 3D pose of the camera (a 4x4 float matrix)
    /// @param[in] camParams the camera parameters
    /// @param[out] worldPoints a set of world 3D points resulting from the unprojection of the 2D image points
    /// @return FrameworkReturnCode::_SUCCESS_ if 3D projection succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode unproject(const std::vector<SolAR::datastructure::Keypoint> & imageKeypoints,
                                  const SolAR::datastructure::Transform3Df & pose,
                                  const SolAR::datastructure::CameraParameters & camParams,
                                  std::vector<SolAR::datastructure::Point3Df> & worldPoints) override;

    void unloadComponent () override final;
private:
    void setCameraParameters(const SolAR::datastructure::CameraParameters & camParams);

    FrameworkReturnCode unprojectOCV(const std::vector<cv::Point2f>& imagePoints,
                                     std::vector<SolAR::datastructure::Point3Df>& worldPoints,
                                     const SolAR::datastructure::Transform3Df& pose);

private:
    cv::Mat m_camMatrix;
    cv::Mat m_camDistorsion;
};

}
}
}

#endif // SOLARUNPROJECTPLANARPOINTSOPENCV_H
