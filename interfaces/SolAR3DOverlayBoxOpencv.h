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
namespace MODULES {
namespace OPENCV {

/**
 * @class SolAR3DOverlayBoxOpencv
 * @brief <B>Draws a 3D box on an image.</B>
 * <TT>UUID: 2db01f59-9793-4cd5-8e13-b25d0ed5735b</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolAR3DOverlayBoxOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::display::I3DOverlay
{
public:
    SolAR3DOverlayBoxOpencv();


    ///@brief draw  a projection of 3D box based on 3D pose on an image.
    /// @param[in] pose: 3D camera pose expressed in the world coordinate.
    /// @param[in,out] displayImage The image on which the projection of a 3D box will be drawn.
    void draw(const datastructure::Transform3Df & pose, SRef<datastructure::Image> displayImage) override;

    /// @brief this method is used to set intrinsic parameters and distorsion of the camera
    /// @param[in] Camera calibration matrix parameters.
    /// @param[in] Camera distorsion parameters.
    void setCameraParameters(const datastructure::CamCalibration & intrinsic_parameters, const datastructure::CamDistortion & distorsion_parameters);

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
