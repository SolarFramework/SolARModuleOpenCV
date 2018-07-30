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

#ifndef SOLAR2DOVERLAYOPENCV_H
#define SOLAR2DOVERLAYOPENCV_H
#include <vector>

#include "api/display/I2DOverlay.h"

#include "xpcf/component/ComponentBase.h"

#include "SolAROpencvAPI.h"

#include "datastructure/Pose.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolAR2DOverlayOpencv : public org::bcom::xpcf::ComponentBase,
    public api::display::I2DOverlay
{
public:
    SolAR2DOverlayOpencv();

    void drawCircle(const SRef<Point2Df> point, const unsigned int radius, const int thickness, const std::vector<unsigned int> & bgrValues, SRef<Image> displayImage) override;

    void drawCircles(const std::vector<SRef<Point2Df>>& points, const unsigned int radius, const int thickness, SRef<Image> displayImage) override;

    /// @brief Draw Circles.
    /// Draw all the circles stored in the vector std::vector <SRef<Keypoint>> & keypoints on image displayImage with specified radius and thickness.
    void drawCircles(const std::vector<SRef<Keypoint>>& keypoints, const unsigned int radius, const int thickness, SRef<Image> displayImage) override;

    void drawContours (const std::vector <SRef<Contour2Df>> & contours, const int thickness, const std::vector<unsigned int> & bgrValues, SRef<Image> displayImage) override;

    void drawSBPattern (const SRef<SquaredBinaryPattern> pattern, SRef<Image> displayImage) override;

    void unloadComponent () override final;

private:

    CamCalibration m_intrinsic_parameters;
    CamDistortion m_distorsion_parameters;


};

}
}
}

#endif // SOLAR2DOVERLAYOPENCV_H
