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

#include "SolARFCNSegmentationOpencv.h"
#include "core/Log.h"
#include "SolAROpenCVHelper.h"
#include <numeric>
#include <algorithm>

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARFCNSegmentationOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARFCNSegmentationOpencv::SolARFCNSegmentationOpencv():ConfigurableBase(xpcf::toUUID<SolARFCNSegmentationOpencv>())
{
    LOG_DEBUG("SolARFCNSegmentationOpencv constructor")
    declareInterface<api::segm::ISemanticSegmentation>(this);
    declareProperty("modelFile", m_modelFile);
    declareProperty("modelConfig", m_modelConfig);
}

SolARFCNSegmentationOpencv::~SolARFCNSegmentationOpencv()
{
    LOG_DEBUG(" SolARFCNSegmentationOpencv destructor")
}

xpcf::XPCFErrorCode SolARFCNSegmentationOpencv::onConfigured()
{
#ifndef __ANDROID__
    LOG_DEBUG(" SolARFCNSegmentationOpencv onConfigured");
	// read and initialize network
	m_net = cv::dnn::readNet(m_modelFile, m_modelConfig);
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
	m_scale = 0.017391f;
	m_mean = cv::Scalar(0.485, 0.456, 0.406) * 255;
	m_inputSize = cv::Size(500, 500);	
	m_outputLayerNames = { "output" };
	return xpcf::XPCFErrorCode::_SUCCESS;
#else
    LOG_ERROR ("SolARYOLACTSegemntationOpencv is not avialble for Android");
    return xpcf::XPCFErrorCode::_FAIL;
#endif
}

FrameworkReturnCode SolARFCNSegmentationOpencv::segment(const SRef<SolAR::datastructure::Image> image,
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
	/// fcn prediction
	// input preparation
	cv::Mat blobInput = cv::dnn::blobFromImage(imageCV, m_scale, m_inputSize, m_mean, true);
	m_net.setInput(blobInput);	
	std::vector<cv::Mat> outs;
	m_net.forward(outs, m_outputLayerNames);
	cv::Mat score = outs[0];	
	/// post-processing
	const int rows = score.size[2];
	const int cols = score.size[3];
	const int chns = score.size[1];
	cv::Mat maxClass = cv::Mat::zeros(rows, cols, CV_8UC1);
	cv::Mat maxVal(rows, cols, CV_32FC1, score.data);
	for (int ch = 1; ch < chns; ch++)
	{
		for (int row = 0; row < rows; row++)
		{
			const float *ptrScore = score.ptr<float>(0, ch, row);
			uint8_t *ptrMaxClass = maxClass.ptr<uint8_t>(row);
			float *ptrMaxVal = maxVal.ptr<float>(row);
			for (int col = 0; col < cols; col++)
			{
				if (ptrScore[col] > ptrMaxVal[col])
				{
					ptrMaxVal[col] = ptrScore[col];
					ptrMaxClass[col] = (uchar)ch;
				}
			}
		}
	}
	cv::Mat maskCV;
	cv::resize(maxClass, maskCV, imageCV.size(), 0.0, 0.0, cv::INTER_NEAREST);
	SolAROpenCVHelper::convertToSolar(maskCV, mask);
	return FrameworkReturnCode::_SUCCESS;
#else
    LOG_ERROR ("SolARYOLACTSegemntationOpencv is not avialble for Android");
    return FrameworkReturnCode::_ERROR_;
#endif
}

}
}
}
