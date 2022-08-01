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

#include "SolARDeepLabV3PlusSegmentationOpencv.h"
#include "core/Log.h"
#include "SolAROpenCVHelper.h"
#include <numeric>
#include <algorithm>

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDeepLabV3PlusSegmentationOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARDeepLabV3PlusSegmentationOpencv::SolARDeepLabV3PlusSegmentationOpencv():ConfigurableBase(xpcf::toUUID<SolARDeepLabV3PlusSegmentationOpencv>())
{
    LOG_DEBUG("SolARDeepLabV3PlusSegmentationOpencv constructor")
    declareInterface<api::segm::ISemanticSegmentation>(this);
    declareProperty("modelFile", m_modelFile);
    declareProperty("modelConfig", m_modelConfig);
	declarePropertySequence("mean", m_mean);
	declarePropertySequence("std", m_std);
}

SolARDeepLabV3PlusSegmentationOpencv::~SolARDeepLabV3PlusSegmentationOpencv()
{
    LOG_DEBUG(" SolARDeepLabV3PlusSegmentationOpencv destructor")
}

xpcf::XPCFErrorCode SolARDeepLabV3PlusSegmentationOpencv::onConfigured()
{
#ifndef __ANDROID__
    LOG_DEBUG(" SolARDeepLabV3PlusSegmentationOpencv onConfigured");
	// read and initialize network
	m_net = cv::dnn::readNet(m_modelFile, m_modelConfig);
	if (std::any_of(m_std.begin(), m_std.end(), [](const auto& v) {return v < 1e-5;})) {
		LOG_ERROR("SolARDeepLabV3PlusSegmentationOpencv normalization std values too small");
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
	m_inputSize = cv::Size(512, 512);	
	m_outputLayerNames = { "output" };
	return xpcf::XPCFErrorCode::_SUCCESS;
#else
    LOG_ERROR ("SolARDeepLabV3PlusSegmentationOpencv is not avialble for Android");
    return xpcf::XPCFErrorCode::_FAIL;
#endif
}

FrameworkReturnCode SolARDeepLabV3PlusSegmentationOpencv::segment(const SRef<SolAR::datastructure::Image> image,
                                                        SRef<SolAR::datastructure::Image> &mask)
{
#ifndef __ANDROID__
	/// convert to opencv image
	cv::Mat imageCV;
	SolAROpenCVHelper::mapToOpenCV(image, imageCV);
	if (imageCV.channels() != 3) {
		LOG_ERROR("Input image must be RGB image");
		return FrameworkReturnCode::_ERROR_;
	}
	/// deeplabv3 prediction
	// input preparation
	cv::Mat blobInput = cv::dnn::blobFromImage(imageCV, 1., m_inputSize, cv::Scalar(m_mean[0], m_mean[1], m_mean[2]), true);
	cv::multiply(blobInput, cv::Scalar(1.f / m_std[0], 1.f / m_std[1], 1.f / m_std[2]), blobInput);
	m_net.setInput(blobInput);
	std::vector<cv::Mat> outs;
	m_net.forward(outs, m_outputLayerNames);
	cv::Mat bufferUchar;
	outs[0].convertTo(bufferUchar, CV_8UC1);
	cv::Mat maskCV(m_inputSize, CV_8UC1, bufferUchar.data);
	cv::resize(maskCV, maskCV, cv::Size(imageCV.cols, imageCV.rows), cv::INTER_NEAREST);
	SolAROpenCVHelper::convertToSolar(maskCV, mask);
	return FrameworkReturnCode::_SUCCESS;
#else
    LOG_ERROR ("SolARDeepLabV3PlusSegmentationOpencv is not avialble for Android");
    return FrameworkReturnCode::_ERROR_;
#endif
}

}
}
}
