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

#ifndef SOLARFCNSEGMENTATIONOPENCV_H
#define SOLARFCNSEGMENTATIONOPENCV_H

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include "api/segm/ISemanticSegmentation.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARFCNSegmentationOpencv
 * @brief <B>Perform 2D semantic segmentation based on the FCN network (Not available for Android!).</B>
 * <TT>UUID: 77a8b776-6b0c-4bc0-b0a8-437a796b8e29</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ m_modelFile,
 *                          the path to the fcn model file,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ m_modelConfig,
 *                          the path to the model configuration file,
 *                          @SolARComponentPropertyDescString{ "" }}
 * 
 */

class SOLAROPENCV_EXPORT_API SolARFCNSegmentationOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::segm::ISemanticSegmentation
{
public:
    SolARFCNSegmentationOpencv();
    ~SolARFCNSegmentationOpencv() override;

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
	cv::dnn::Net	m_net;
	float			m_scale;	
	cv::Scalar		m_mean;
	cv::Size		m_inputSize;
	std::vector<std::string> m_outputLayerNames;
};

}
}
}  // end of namespace Solar

#endif // SOLARFCNSEGMENTATIONOPENCV_H
