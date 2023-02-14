/**
 * @copyright Copyright (c) 2020 B-com http://www.b-com.com/
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

#ifndef SOLARMASKOVERLAYOPENCV_H
#define SOLARMASKOVERLAYOPENCV_H

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include "api/display/IMaskOverlay.h"
#include <opencv2/opencv.hpp>

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARMaskOverlayOpencv
 * @brief <B>Draws masks on top of an image.</B>
 * <TT>UUID: ed445504-daba-4855-af88-052d4e3e5b7a</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ m_classFile,
 *                          the path to class name file,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ m_colorFile,
 *                          the path to the color file,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentPropertiesEnd
 * 
 */

class SOLAROPENCV_EXPORT_API SolARMaskOverlayOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::display::IMaskOverlay
{
public:
    SolARMaskOverlayOpencv();
    ~SolARMaskOverlayOpencv() override;

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;
    void unloadComponent () override;

    /// @brief Draw masks on an image.
    /// @param[in,out] image The image on which the masks will be drawn.
    /// @param[in] boxes The bounding boxes of each detected object.
    /// @param[in] masks The binary masks corresponding to the bounding boxes. For each mask, regions with a value of 1 correspond to the object, otherwise the background.
    /// @param[in] classIds The id of each object in the bounding box.
    /// @param[in] scores The corresponding confidence scores.
	/// @return FrameworkReturnCode::_SUCCESS if the draw succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode draw(SRef<SolAR::datastructure::Image> image,
                             const std::vector<SolAR::datastructure::Rectanglei> &boxes,
                             const std::vector<SRef<SolAR::datastructure::Image>> &masks,
                             const std::vector<uint32_t> &classIds,
                             const std::vector<float> &scores) override;

	/// @brief Draw masks on an image.
	/// @param[in,out] image The image on which the masks will be drawn.
	/// @param[in] mask The mask has same size as the input image, in which the value of each pixel is corresponding to the class id.
	/// @return FrameworkReturnCode::_SUCCESS if the draw succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode draw(SRef<SolAR::datastructure::Image> image,
							 const SRef<SolAR::datastructure::Image> mask) override;

private:
       std::string                 m_classFile;
       std::string                 m_colorFile;
       std::vector<std::string>    m_classes;
       std::vector<cv::Scalar>     m_colors;
       std::vector<int>            m_otherClassColor = {0, 0, 0};
       std::vector<std::string>    m_classes_legend;
       std::vector<cv::Scalar>     m_colors_legend;
};

}
}
}  // end of namespace Solar

#endif // SOLARMASKOVERLAYOPENCV_H
