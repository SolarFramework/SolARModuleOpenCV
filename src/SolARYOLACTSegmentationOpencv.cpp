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

#include "SolARYOLACTSegmentationOpencv.h"
#include "core/Log.h"
#include "SolAROpenCVHelper.h"
#include <numeric>
#include <algorithm>

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARYOLACTSegmentationOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARYOLACTSegmentationOpencv::SolARYOLACTSegmentationOpencv():ConfigurableBase(xpcf::toUUID<SolARYOLACTSegmentationOpencv>())
{
    LOG_DEBUG("SolARYOLACTSegmentationOpencv constructor")
    declareInterface<api::segm::IInstanceSegmentation>(this);
    declareProperty("modelFile", m_modelFile);
    declareProperty("modelConfig", m_modelConfig);
    declareProperty("confThresh", m_confThresh);
    declareProperty("maskThresh", m_maskThresh);
}

SolARYOLACTSegmentationOpencv::~SolARYOLACTSegmentationOpencv()
{
    LOG_DEBUG(" SolARYOLACTSegmentationOpencv destructor")
}

xpcf::XPCFErrorCode SolARYOLACTSegmentationOpencv::onConfigured()
{
	LOG_DEBUG(" SolARYOLACTSegmentationOpencv onConfigured");
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
	m_scale = 0.017391;
	m_mean = cv::Scalar(123.675, 116.78, 103.94);
	m_inputSize = cv::Size(550, 550);	
	m_maskSize = cv::Size(138, 138);
	m_nbChannels = 32;
	m_outputLayerNames = { "conf", "mask", "proto", "boxes" };
	return xpcf::XPCFErrorCode::_SUCCESS;
}

bool checkJaccard(const Rectanglei& box, const uint32_t& id, const std::vector<Rectanglei>& boxes, const std::vector<uint32_t> &ids)
{
	for (int i = 0; i < boxes.size(); ++i) {
		if (id == ids[i]) {
			float dx = std::min(box.startX + box.size.width - 1, boxes[i].startX + boxes[i].size.width - 1) - 
				std::max(box.startX, boxes[i].startX);
			float dy = std::min(box.startY + box.size.height - 1, boxes[i].startY + boxes[i].size.height - 1) - 
				std::max(box.startY, boxes[i].startY);
			if ((dx > 0) && (dy > 0)) {
				float intersect = dx * dy;
				float unionBox = box.size.width * box.size.height + boxes[i].size.width * boxes[i].size.height - intersect;
				if (intersect / unionBox > 0.5f)
					return false;
			}
		}
	}
	return true;
}

FrameworkReturnCode SolARYOLACTSegmentationOpencv::segment(const SRef<SolAR::datastructure::Image> image,
                                                           std::vector<SolAR::datastructure::Rectanglei> &boxes,
                                                           std::vector<SRef<SolAR::datastructure::Image>> &masks,
                                                           std::vector<uint32_t> &classIds,
                                                           std::vector<float> &scores)
{
	boxes.clear();
	masks.clear();
	classIds.clear();
	scores.clear();
	/// convert to opencv image
	cv::Mat imageCV;
	SolAROpenCVHelper::mapToOpenCV(image, imageCV);
	if (imageCV.channels() != 3) {
		LOG_ERROR("Input image must be RGB image");
		return FrameworkReturnCode::_ERROR_;
	}
	/// yolact prediction
	// input preparation
	cv::Mat blobInput = cv::dnn::blobFromImage(imageCV, m_scale, m_inputSize, m_mean, true);
	m_net.setInput(blobInput);	
	std::vector<cv::Mat> outs;
	m_net.forward(outs, m_outputLayerNames);
	// get prediction
	int nbDetections = outs[0].size[1];
	int nbClasses = outs[0].size[2];
	// reshape outputs
	cv::Mat outConfs = cv::Mat(nbDetections, nbClasses, CV_32FC1, outs[0].data);
	cv::Mat outMasks = cv::Mat(nbDetections, m_nbChannels, CV_32FC1, outs[1].data);
	cv::Mat outProto = cv::Mat(m_maskSize.height * m_maskSize.width, m_nbChannels, CV_32FC1, outs[2].data);
	cv::Mat outBoxes = cv::Mat(nbDetections, 4, CV_32FC1, outs[3].data);
	LOG_DEBUG("Number of detections: {}", nbDetections);
	LOG_DEBUG("Number of classes: {}", nbClasses);
	/// post-processing
	float W = image->getWidth();
	float H = image->getHeight();
	std::vector<float> filteredConfs;
	std::vector<uint32_t> filteredClassIds;
	std::vector<cv::Mat> filteredMasks;
	std::vector<Rectanglei> filteredBoxes;
	// filter outputs
	for (int i = 0; i < nbDetections; ++i) {
		double minVal, maxVal;
		cv::Point minIdx, maxIdx;
		cv::minMaxLoc(outConfs.row(i), &minVal, &maxVal, &minIdx, &maxIdx);
		if ((maxIdx.x == 0) || (maxVal < m_confThresh))
			continue;
		filteredConfs.push_back(maxVal);
		filteredClassIds.push_back(maxIdx.x - 1);
		filteredMasks.push_back(outMasks.row(i));
		// extract box
		float l = std::max(0.f, std::min(W * outBoxes.at<float>(i, 0), W - 1));
		float t = std::max(0.f, std::min(H * outBoxes.at<float>(i, 1), H - 1));
		float r = std::max(0.f, std::min(W * outBoxes.at<float>(i, 2), W - 1));
		float b = std::max(0.f, std::min(H * outBoxes.at<float>(i, 3), H - 1));
		Rectanglei box;
		box.startX = (int)l;
		box.startY = (int)t;
		box.size.width = (int)r - box.startX + 1;
		box.size.height = (int)b - box.startY + 1;
		filteredBoxes.push_back(box);
	}
	LOG_DEBUG("Number of filtered detections: {}", filteredBoxes.size());
	// sort confidence score
	std::vector<size_t> idx(filteredConfs.size());
	std::iota(idx.begin(), idx.end(), 0);
	std::sort(idx.begin(), idx.end(),
		[&filteredConfs](size_t i1, size_t i2) {return filteredConfs[i1] > filteredConfs[i2]; });
	// get best outputs
	for (auto i : idx) {
		Rectanglei box = filteredBoxes[i];
		if (checkJaccard(box, filteredClassIds[i], boxes, classIds)) {			
			// find mask
			cv::Mat maskData = outProto * filteredMasks[i].t();
			cv::Mat objectMask(m_maskSize.height, m_maskSize.width, CV_32FC1, maskData.data);
			cv::resize(objectMask, objectMask, imageCV.size());
			cv::Rect rect(box.startX, box.startY, box.size.width, box.size.height);
			cv::Mat cropMask = objectMask(rect);
			cv::Mat maskCV = (cropMask > m_maskThresh);
			SRef<Image> maskSolAR;
			SolAROpenCVHelper::convertToSolar(maskCV, maskSolAR);
			// add output
			boxes.push_back(box);
			classIds.push_back(filteredClassIds[i]);
			scores.push_back(filteredConfs[i]);
			masks.push_back(maskSolAR);
		}
	}
	LOG_DEBUG("Number of output detections: {}", boxes.size());
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}
