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

#ifndef SOLARUNDISTORTPOINTS_H
#define SOLARUNDISTORTPOINTS_H
#include <vector>

#include "xpcf/component/ComponentBase.h"
#include "SolAROpencvAPI.h"
#include "api/geom/IUndistortPoints.h"

#include "opencv2/core.hpp"

#include "SolAROpencvAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARUndistortPointsOpencv : public org::bcom::xpcf::ComponentBase,
    public api::geom::IUndistortPoints
{
public:
    SolARUndistortPointsOpencv();
    ~SolARUndistortPointsOpencv() = default;

    void unloadComponent () override final;
    FrameworkReturnCode undistort(const std::vector<SRef<Point2Df>> & inputPoints, std::vector<SRef<Point2Df>> & outputPoints) override;

    /// @brief Set the distorsion intrinsic camera parameters
    void setDistorsionParameters(const CamDistortion & distorsion_parameters) override;
        /// @brief Set the intrinsic camera parameters
    void setIntrinsicParameters(const CamCalibration & intrinsic_parameters) override;
    

private:
    CamCalibration m_intrinsic_parameters;
    CamDistortion m_distorsion_parameters;

    cv::Mat m_camMatrix;
    cv::Mat m_camDistorsion;
};

}
}
}

#endif // SOLARUNDISTORTPOINTS_H
