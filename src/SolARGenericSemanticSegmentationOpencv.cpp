/**
 * @copyright Copyright (c) 2023 B-com http://www.b-com.com/
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

#include "SolARGenericSemanticSegmentationOpencv.h"
#include "core/Log.h"
#include "SolAROpenCVHelper.h"
#include <numeric>
#include <algorithm>

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARGenericSemanticSegmentationOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARGenericSemanticSegmentationOpencv::SolARGenericSemanticSegmentationOpencv():ConfigurableBase(xpcf::toUUID<SolARGenericSemanticSegmentationOpencv>())
{
    LOG_DEBUG("SolARGenericSemanticSegmentationOpencv constructor")
    declareInterface<api::segm::ISemanticSegmentation>(this);
    declareProperty("modelFile", m_modelFile);
    declareProperty("argMaxRemoved", m_argMaxRemoved);
    declareProperty("modelConfig", m_modelConfig);
    declarePropertySequence("mean", m_mean);
    declarePropertySequence("std", m_std);
    declarePropertySequence("inputSize", m_inputSize);
}

SolARGenericSemanticSegmentationOpencv::~SolARGenericSemanticSegmentationOpencv()
{
    LOG_DEBUG("SolARGenericSemanticSegmentationOpencv destructor")
}

xpcf::XPCFErrorCode SolARGenericSemanticSegmentationOpencv::onConfigured()
{
#ifndef __ANDROID__
    LOG_DEBUG("SolARGenericSemanticSegmentationOpencv onConfigured");
    // read and initialize network
    m_net = cv::dnn::readNet(m_modelFile, m_modelConfig);
    if (std::any_of(m_std.begin(), m_std.end(), [](const auto& v) {return v < 1e-5;})) {
        LOG_ERROR("SolARGenericSemanticSegmentationOpencv normalization standard deviation values too small");
        return xpcf::XPCFErrorCode::_FAIL;
    }
#ifdef WITHCUDA
    LOG_INFO("Using GPU device");
    m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
#else
    LOG_INFO("Using CPU device");
    m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
#endif // WITHCUDA
    // set network parameters	
    m_outputLayerNames = { "output" };
    return xpcf::XPCFErrorCode::_SUCCESS;
#else
    LOG_ERROR("SolARGenericSemanticSegmentationOpencv is not available for Android");
    return xpcf::XPCFErrorCode::_FAIL;
#endif
}

FrameworkReturnCode SolARGenericSemanticSegmentationOpencv::segment(const SRef<SolAR::datastructure::Image> image, SRef<SolAR::datastructure::Image>& mask)
{
#ifndef __ANDROID__
    /// convert to opencv image
    cv::Mat imageCV;
    SolAROpenCVHelper::mapToOpenCV(image, imageCV);
    if (imageCV.channels() != 3) {
        LOG_ERROR("Input image must be RGB image");
        return FrameworkReturnCode::_ERROR_;
    }
    /// mmsegmentation model prediction
    // input preparation
    cv::Size inputImageSize(m_inputSize[0], m_inputSize[1]);
    cv::Mat blobInput = cv::dnn::blobFromImage(imageCV, 1., inputImageSize, cv::Scalar(m_mean[0], m_mean[1], m_mean[2]), true);
    cv::multiply(blobInput, cv::Scalar(1.f / m_std[0], 1.f / m_std[1], 1.f / m_std[2]), blobInput);
    m_net.setInput(blobInput);
    std::vector<cv::Mat> outs;
    m_net.forward(outs, m_outputLayerNames);
    cv::Mat bufferUchar;
    if (m_argMaxRemoved > 0) {
        int numPixels = m_inputSize[0] * m_inputSize[1];
        int numClasses = outs[0].total() / numPixels;
        bufferUchar = cv::Mat::zeros(1, numPixels, CV_8UC1);
        float* buffer = outs[0].ptr<float>();
        for (int h = 0; h < m_inputSize[1]; h++) {
            for (int w = 0; w < m_inputSize[0]; w++) {
                auto curIdx = h * m_inputSize[0] + w;
                // assume that class with max proba value is 0 
                int idxmax = 0;
                float valuemax = buffer[curIdx];
                // loop over all other classes to find class with bigger proba value  
                for (int c = 1; c < numClasses; c++) {
                    float valueclass = buffer[c * numPixels + curIdx];
                    if (valueclass > valuemax) {
                        idxmax = c;
                        valuemax = valueclass;
                    }
                }
                bufferUchar.at<unsigned char>(0, curIdx) = static_cast<unsigned char>(idxmax);
            }
        }
    }
    else {
        outs[0].convertTo(bufferUchar, CV_8UC1);
    }
    cv::Mat maskCV(inputImageSize, CV_8UC1, bufferUchar.data);
    // resize back to original image size 
    cv::resize(maskCV, maskCV, cv::Size(imageCV.cols, imageCV.rows),0., 0., cv::INTER_NEAREST);
    SolAROpenCVHelper::convertToSolar(maskCV, mask);
    return FrameworkReturnCode::_SUCCESS;
#else
    LOG_ERROR("SolARGenericSemanticSegmentationOpencv is not available for Android");
    return FrameworkReturnCode::_ERROR_;
#endif
}

}
}
}
