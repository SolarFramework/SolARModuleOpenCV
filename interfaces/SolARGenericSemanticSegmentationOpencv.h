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

#ifndef SOLARGENERICSEMANTICSEGMENTATIONOPENCV_H
#define SOLARGENERICSEMANTICSEGMENTATIONOPENCV_H

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include "api/segm/ISemanticSegmentation.h"
#include <opencv2/opencv.hpp>
#ifndef __ANDROID__
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#endif

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARGenericSemanticSegmentationOpencv
 * @brief <B>Perform 2D generic semantic segmentation based on neural network.</B>
 * <TT>UUID: 063d3086-6c87-49ff-87db-9a4ff224b5c8</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ m_modelFile,
 *                          the path to the DeepLabV3+ model file,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ m_modelConfig,
 *                          the path to the model configuration file,
 *                          @SolARComponentPropertyDescString{ "" }}
 * 
 */

class SOLAROPENCV_EXPORT_API SolARGenericSemanticSegmentationOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::segm::ISemanticSegmentation
{
public:
    SolARGenericSemanticSegmentationOpencv();
    ~SolARGenericSemanticSegmentationOpencv() override;

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;
    void unloadComponent () override;

    /// @brief Perform 2D semantic segmentation
    /// @param[in] image The input image.
    /// @param[out] mask The mask has same size as the input image, in which the value of each pixel is corresponding to the class id.
    /// @return FrameworkReturnCode::_SUCCESS if the segmentation succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode segment(const SRef<SolAR::datastructure::Image> image,
                                SRef<SolAR::datastructure::Image> &mask) override;

private:
    std::string		m_modelFile = "";
    std::string		m_modelConfig = "";
#ifndef __ANDROID__
    cv::dnn::Net	m_net;
#endif
    std::vector<float>	m_std = {0.f, 0.f, 0.f};
    std::vector<float>	m_mean = {0.f, 0.f, 0.f};
    std::vector<int>	m_inputSize = {0, 0};  // width, height
    std::vector<std::string> m_outputLayerNames;
    int m_argMaxRemoved = 0;
};

}
}
}  // end of namespace Solar

#endif // SOLARGENERICSEMANTICSEGMENTATIONOPENCV_H
