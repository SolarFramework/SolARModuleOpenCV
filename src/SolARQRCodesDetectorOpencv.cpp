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

#include "SolARQRCodesDetectorOpencv.h"
#include "core/Log.h"
#include "SolAROpenCVHelper.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARQRCodesDetectorOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARQRCodesDetectorOpencv::SolARQRCodesDetectorOpencv():ConfigurableBase(xpcf::toUUID<SolARQRCodesDetectorOpencv>())
{
    addInterface<SolAR::api::features::I2DTrackablesDetector>(this);
    declareInjectable<SolAR::api::image::IImageConvertor>(m_imageConvertor);
    declareInjectable<SolAR::api::features::ICornerRefinement>(m_cornerRefinement);
    LOG_DEBUG("SolARMultiQRCodesPoseEstimatorOpencv constructor");
}

FrameworkReturnCode SolARQRCodesDetectorOpencv::setTrackables(const std::vector<SRef<SolAR::datastructure::Trackable>> trackables)
{
	m_nbMarkers = trackables.size();
	if (m_nbMarkers == 0)
		return FrameworkReturnCode::_ERROR_;
	    
    for (const auto &trackable : trackables)
        if (trackable->getType() == TrackableType::QRCODE_MARKER) {
			SRef<QRCode> qrCode = xpcf::utils::dynamic_pointer_cast<QRCode>(trackable);
			m_QRCodes.push_back(qrCode);
			LOG_DEBUG("Decoding code: {}", qrCode->getCode());			
		}
        else {
            LOG_ERROR("The SolARMultiQRCodesPoseEstimatorOpencv should only use a trackable of type QRCODE")
            return FrameworkReturnCode::_ERROR_;
        }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARQRCodesDetectorOpencv::detect(const SRef<SolAR::datastructure::Image> image,
                                                         std::vector<std::vector<SolAR::datastructure::Point2Df>> & corners)
{    
    if (m_nbMarkers == 0){
        LOG_ERROR("Must set QR codes before detect");
        return FrameworkReturnCode::_ERROR_;
    }
    // init corners
    corners.resize(m_nbMarkers);

    SRef<Image>				greyImage;
    std::vector<Point2Df>	pts2D;
    int                     nbFoundMarkers(0);

    // Convert Image from RGB to grey
    if (image->getNbChannels() != 1)
        m_imageConvertor->convert(image, greyImage, Image::ImageLayout::LAYOUT_GREY);
    else
        greyImage = image->copy();

	cv::Mat cvImage = SolAROpenCVHelper::mapToOpenCV(greyImage);
    std::vector<cv::Point2f> detectedCorners;
	std::vector<std::string> codes;
	std::vector<cv::Mat> rectifiedImage;
    m_qrDetector.detectAndDecodeMulti(cvImage, codes, detectedCorners, rectifiedImage);

	for (int i = 0; i < m_nbMarkers; ++i) {
        std::vector<cv::Point2f> img2Dpts;
		for (int j = 0; j < codes.size(); ++j)
			if (m_QRCodes[i]->getCode() == codes[j]) {
				// get 2D points
                img2Dpts.assign(detectedCorners.begin() + j * 4, detectedCorners.begin() + j * 4 + 4);
				nbFoundMarkers++;
				break;
			}
        if (img2Dpts.size() == 4) {
            for (int k = 0; k < 4; k++) {
                std::vector<Point2Df> pts{Point2Df(img2Dpts[k].x, img2Dpts[k].y)};
                if (k != 2)
                    m_cornerRefinement->refine(greyImage, pts);
                corners[i].push_back(pts[0]);
            }
        }
	}

	LOG_DEBUG("Number of detected markers: {}", nbFoundMarkers);

	if (nbFoundMarkers == 0)
		return FrameworkReturnCode::_ERROR_;
    else
        return FrameworkReturnCode::_SUCCESS;
}

}
}
}
