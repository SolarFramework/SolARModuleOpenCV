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

#ifndef SOLARSTEREO2DPOINTSRECTIFICATIONOPENCV_H
#define SOLARSTEREO2DPOINTSRECTIFICATIONOPENCV_H

#include "base/geom/A2DPointsRectification.h"
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARStereo2DPointsRectificationOpencv
* @brief <B>Rectify 2D points.</B>
* <TT>UUID: bf4c7011-b7e6-453d-a755-884dac18d3ee</TT>
*
*/

class SOLAROPENCV_EXPORT_API SolARStereo2DPointsRectificationOpencv : public base::geom::A2DPointsRectification
{
public:
    /// @brief SolARStereo2DPointsRectificationOpencv constructor
    SolARStereo2DPointsRectificationOpencv();

    /// @brief SolARStereo2DPointsRectificationOpencv destructor
    ~SolARStereo2DPointsRectificationOpencv() override;

    /// @brief Rectify 2D points
    /// @param[in] points2D The input 2D points
    /// @param[in] camParams The camera parameters of camera
    /// @param[in] rectParams The rectification parameters of camera
    /// @param[out] rectifiedPoints2D The rectified 2D points
    /// @return FrameworkReturnCode::_SUCCESS if rectifying succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode rectify(const std::vector<SolAR::datastructure::Point2Df>& points2D,
                                const SolAR::datastructure::CameraParameters& camParams,
                                const SolAR::datastructure::RectificationParameters& rectParams,
                                std::vector<SolAR::datastructure::Point2Df>& rectifiedPoints2D) override;

	void unloadComponent() override;
};
}
}
}  // end of namespace Solar
#endif // SOLARSTEREO2DPOINTSRECTIFICATIONOPENCV_H
