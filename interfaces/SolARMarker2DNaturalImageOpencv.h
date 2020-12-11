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

#ifndef SOLARMARKER2DNATURALIMAGEOPENCV_H
#define SOLARMARKER2DNATURALIMAGEOPENCV_H

#include "api/input/files/IMarker2DNaturalImage.h"

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include "opencv2/opencv.hpp"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARMarker2DNaturalImageOpencv
 * @brief <B>Loads a 2D natural image marker from a file.</B>
 * <TT>UUID: efcdb590-c570-11e7-abc4-cec278b6b50a</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARMarker2DNaturalImageOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::input::files::IMarker2DNaturalImage {
public:
    SolARMarker2DNaturalImageOpencv();

    ~SolARMarker2DNaturalImageOpencv() override = default;
    void unloadComponent () override final;

    FrameworkReturnCode loadMarker() override;
    FrameworkReturnCode getImage(SRef<datastructure::Image> & img) override;

    /// @brief Provide the position of 2D corners in image coordinate system
    /// @param[out] imageCorners the 2D corners of the marker in image coordinate system
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode getImageCorners(std::vector<datastructure::Point2Df> & imageCorners) const override;

    /// @brief Provide the position of 3D corners in world coordinate system
    /// @param[out] worldCorners the 3D corners of the marker in world coordinate system
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode getWorldCorners(std::vector<datastructure::Point3Df> & worldCorners) const override;

    void setSize (const float & width, const float & height) override { m_size.width = width; m_size.height = height; }
    float getWidth() const override { return m_size.width; }
    float getHeight() const override { return m_size.height; }
    const datastructure::Sizef & getSize() const override { return m_size; }

private:
    datastructure::Sizef m_size;
    cv::Mat m_ocvImage;

    /// @brief the path to the file describing the 2D Squared binary marker
    std::string m_filePath ="";
};

}
}
}

#endif
