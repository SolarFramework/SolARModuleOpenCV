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

#ifndef SolAR3DOverlayBoxOpencv_H
#define SolAR3DOverlayBoxOpencv_H
#include <vector>

#include "api/display/I3DOverlay.h"

#include "opencv2/core.hpp"

#include "SolAROpencvAPI.h"

#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolAR3DOverlayBoxOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::display::I3DOverlay
{
public:
    SolAR3DOverlayBoxOpencv();

    void draw(const Transform3Df & pose, SRef<Image> displayImage) override;

    void setCameraParameters(const CamCalibration & intrinsic_parameters, const CamDistortion & distorsion_parameters);

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

    void unloadComponent () override final;

private:
    /// @brief position of the center of the bottom face of the Box defined in world unit
    std::vector<float> m_position = {0.0,0.0,0.0};

    /// @brief orientation of the box in euler angles in degrees
    std::vector<float> m_orientation = {0.0, 0.0, 0.0};

    /// @brief size of the box define in world unit
    std::vector<float> m_size = {1.0,1.0,1.0};


    cv::Mat m_camMatrix;
    cv::Mat m_camDistorsion;

    cv::Mat m_parallelepiped; // volume to display

};

}
}
}

#endif // SolAR3DOverlayBoxOpencv_H
