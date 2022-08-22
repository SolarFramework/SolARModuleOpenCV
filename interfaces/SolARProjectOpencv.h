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

#ifndef SOLARPROJECTOPENCV_H
#define SOLARPROJECTOPENCV_H
#include <vector>
#include "opencv2/core.hpp"
#include "api/geom/IProject.h"
#include "SolAROpencvAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARProjectOpencv
* @brief <B>Projects a set of 3D points on a 2D image plane.</B>
* <TT>UUID: 741fc298-0149-4322-a7a9-ccb971e857ba</TT>
*
*/

class SOLAROPENCV_EXPORT_API SolARProjectOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::geom::IProject
{
public:
    ///@brief SolARProjectOpencv constructor;
    SolARProjectOpencv();
    ///@brief SolARProjectOpencv destructor;
    ~SolARProjectOpencv() override;

    /// @brief This method project a set of 3D points in the image plane
    /// @param[in] inputPoints the set of 3D points to project
    /// @param[in] pose the 3D pose of the camera (a 4x4 float matrix)
    /// @param[in] camParams the camera parameters.
    /// @param[out] imagePoints the resulting set of 2D points defined in the image coordinate systemn
    /// @return FrameworkReturnCode::_SUCCESS_ if 3D projection succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode project(const std::vector<SolAR::datastructure::Point3Df> & inputPoints,
                                const SolAR::datastructure::Transform3Df& pose,
                                const SolAR::datastructure::CameraParameters & camParams,
                                std::vector<SolAR::datastructure::Point2Df> & imagePoints) override;

    /// @brief This method project a set of 3D cloud points in the image plane
    /// @param[in] inputPoints the set of 3D cloud points to project
    /// @param[in] pose the 3D pose of the camera (a 4x4 float matrix)
    /// @param[in] camParams the camera parameters.
    /// @param[out] imagePoints the resulting set of 2D points defined in the image coordinate systemn
    /// @return FrameworkReturnCode::_SUCCESS_ if 3D projection succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode project(const std::vector<SRef<SolAR::datastructure::CloudPoint>> & inputPoints,
                                const SolAR::datastructure::Transform3Df& pose,
                                const SolAR::datastructure::CameraParameters & camParams,
                                std::vector<SolAR::datastructure::Point2Df> & imagePoints) override;


    void unloadComponent () override final;

private:
	void setCameraParameters(const SolAR::datastructure::CameraParameters & camParams);

private:
    cv::Mat m_camMatrix;
    cv::Mat m_camDistorsion;
};

}
}
}

#endif // SOLARPROJECTOPENCV_H
