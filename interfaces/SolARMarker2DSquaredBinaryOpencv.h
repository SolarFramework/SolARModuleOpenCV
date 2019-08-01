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

#ifndef SOLARMARKER2DSQUAREDBINARYOPENCV_H
#define SOLARMARKER2DSQUAREDBINARYOPENCV_H

#include "api/input/files/IMarker2DSquaredBinary.h"

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARMarker2DSquaredBinaryOpencv
 * @brief <B>Loads a 2D squared binary marker from a file.</B>
 * <TT>UUID: 5d2b8da9-528e-4e5e-96c1-f883edcf3b1c</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARMarker2DSquaredBinaryOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::input::files::IMarker2DSquaredBinary{
public:

   SolARMarker2DSquaredBinaryOpencv();
   ~SolARMarker2DSquaredBinaryOpencv() override;

    FrameworkReturnCode loadMarker() override;

    /// @brief Provide the position of 2D corners in image coordinate system
    /// @param[out] imageCorners the 2D corners of the marker in image coordinate system
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode getImageCorners(std::vector<Point2Df> & imageCorners) const override;

    /// @brief Provide the position of 3D corners in world coordinate system
    /// @param[out] worldCorners the 3D corners of the marker in world coordinate system
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode getWorldCorners(std::vector<Point3Df> & worldCorners) const override;

    void setSize (const float & width, const float & height) override { m_size.width = width; m_size.height = height; }
    float getWidth() const override { return m_size.width; }
    float getHeight() const override { return m_size.height; }
    const Sizef & getSize() const override { return m_size; }

    const SquaredBinaryPattern & getPattern() const override { return m_pattern; }

protected:
    Sizef m_size{}; ///<define the size of the 2D Marker according to the user-defined unit (the same used for the camera calibration)
    SquaredBinaryPattern m_pattern;

    void unloadComponent () final;

 private:
    /// @brief the path to the file describing the 2D Squared binary marker
    std::string m_filePath ="";

};

}
}
}  // end of namespace Solar
#endif // SOLARMARKER2DSQUAREDBINARYOPENCV_H



