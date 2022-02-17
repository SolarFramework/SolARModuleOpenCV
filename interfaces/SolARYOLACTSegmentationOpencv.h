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

#ifndef SOLARYOLACTSEGMENTATIONOPENCV_H
#define SOLARYOLACTSEGMENTATIONOPENCV_H

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include "api/segm/IInstanceSegmentation.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARYOLACTSegmentationOpencv
 * @brief <B>Perform 2D instance segmentation based on the YOLACT network.</B>
 * <TT>UUID: b6288dde-4e58-4ead-8e41-f2ce98f43626</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ m_modelFile,
 *                          the path to the yolact model file,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ m_modelConfig,
 *                          the path to the model configuration file,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ m_confThresh,
 *                          the confidence threshold,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.5f }}
 * @SolARComponentProperty{ m_maskThresh,
 *                          the mask threshold,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.f }}
 * @SolARComponentPropertiesEnd
 * 
 */

class SOLAROPENCV_EXPORT_API SolARYOLACTSegmentationOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::segm::IInstanceSegmentation
{
public:
    SolARYOLACTSegmentationOpencv();
    ~SolARYOLACTSegmentationOpencv() override;

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;
    void unloadComponent () override;

    /// @brief Perform 2D instance segmentation
    /// @param[in] image The input image.
    /// @param[out] boxes The bounding boxes of each detected object.
    /// @param[out] masks The binary masks corresponding to the bounding boxes. For each mask, regions with a value of 1 correspond to the object, otherwise the background.
    /// @param[out] classIds The id of each object in the bounding box.
    /// @param[out] scores The corresponding confidence scores.
    /// @return FrameworkReturnCode::_SUCCESS if the segmentation succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode segment(const SRef<SolAR::datastructure::Image> image,
                                std::vector<SolAR::datastructure::Rectanglei> &boxes,
                                std::vector<SRef<SolAR::datastructure::Image>> &masks,
                                std::vector<uint32_t> &classIds,
                                std::vector<float> &scores) override;

private:
    float			m_confThresh = 0.5f;
    float			m_maskThresh = 0.f;
    std::string		m_modelFile = "";
    std::string		m_modelConfig = "";
	cv::dnn::Net	m_net;
	float			m_scale;	
	cv::Scalar		m_mean;
	cv::Size		m_inputSize;
	cv::Size		m_maskSize;
	uint32_t		m_nbChannels;
	std::vector<std::string> m_outputLayerNames;
};

}
}
}  // end of namespace Solar

#endif // SOLARYOLACTSEGMENTATIONOPENCV_H
