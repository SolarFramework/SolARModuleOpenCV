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

#ifndef SOLAR3DOVERLAYOPENCV_H
#define SOLAR3DOVERLAYOPENCV_H
#include <vector>

#include "api/display/I3DOverlay.h"

#include "opencv2/core.hpp"

#include "SolAROpencvAPI.h"

#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolAR3DOverlayOpencv : public org::bcom::xpcf::ComponentBase,
    public api::display::I3DOverlay
{
public:
    SolAR3DOverlayOpencv();

    void drawBox (const Transform3Df & pose, const float X_world, const float Y_world, const float Z_world, const Transform3Df affineTransform, SRef<Image> displayImage) override;

     void setCameraParameters(const CamCalibration & intrinsic_parameters, const CamDistortion & distorsion_parameters);

    void unloadComponent () override final;

private:

    void setParallelepipedPosition(const float X_world, const float Y_world, const float Z_world);
    void moveParalleliped(const Transform3Df transformation);
    FrameworkReturnCode transform3D(const std::vector<SRef<Point3Df>> & inputPoints, const Transform3Df transformation, std::vector<SRef<Point3Df>> & outputPoints);

    CamCalibration m_intrinsic_parameters;
    CamDistortion m_distorsion_parameters;

    cv::Mat m_camMatrix;
    cv::Mat m_camDistorsion;

    cv::Mat m_parallelepiped; // volume to display

};

}
}
}

#endif // SOLAR3DOVERLAYOPENCV_H
