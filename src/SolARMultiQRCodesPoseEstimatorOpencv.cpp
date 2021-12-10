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

#include "SolARMultiQRCodesPoseEstimatorOpencv.h"
#include "core/Log.h"
#include "SolAROpenCVHelper.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARMultiQRCodesPoseEstimatorOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARMultiQRCodesPoseEstimatorOpencv::SolARMultiQRCodesPoseEstimatorOpencv():ConfigurableBase(xpcf::toUUID<SolARMultiQRCodesPoseEstimatorOpencv>())
{
    addInterface<SolAR::api::solver::pose::IMultiTrackablesPose>(this);
    declareInjectable<SolAR::api::image::IImageConvertor>(m_imageConvertor);
    declareInjectable<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>(m_pnp);
    declareInjectable<SolAR::api::features::ICornerRefinement>(m_cornerRefinement);
    declareInjectable<SolAR::api::geom::IProject>(m_projector);
    declareProperty("maxReprojError", m_maxReprojError);
    LOG_DEBUG("SolARMultiQRCodesPoseEstimatorOpencv constructor");
}

void SolARMultiQRCodesPoseEstimatorOpencv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
    m_camMatrix = intrinsicParams;
    m_camDistortion = distortionParams;
    m_pnp->setCameraParameters(m_camMatrix, m_camDistortion);
    m_projector->setCameraParameters(m_camMatrix, m_camDistortion);
}

FrameworkReturnCode SolARMultiQRCodesPoseEstimatorOpencv::setTrackables(const std::vector<SRef<SolAR::datastructure::Trackable>> trackables)
{
	m_nbMarkers = trackables.size();
	if (m_nbMarkers == 0)
		return FrameworkReturnCode::_ERROR_;
	
    if (trackables[0]->getType() == TrackableType::QRCODE_MARKER)
    {
		for (const auto &trackable : trackables) {
			SRef<QRCode> qrCode = xpcf::utils::dynamic_pointer_cast<QRCode>(trackable);
			m_QRCodes.push_back(qrCode);
			LOG_DEBUG("Decoding code: {}", qrCode->getCode());			
			// get 3D pattern points
			std::vector<Point3Df> pts3D;
			qrCode->getWorldCorners(pts3D);
			m_pattern3DPoints.push_back(pts3D);
		}
    }
    else {
        LOG_ERROR("The SolARMultiQRCodesPoseEstimatorOpencv should only use a trackable of type QRCODE")
        return FrameworkReturnCode::_ERROR_;
    }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMultiQRCodesPoseEstimatorOpencv::estimate(const SRef<Image> image, Transform3Df & pose)
{
    SRef<Image>				greyImage;    
    std::vector<Point2Df>	pts2D;
    std::vector<Point3Df>	pts3D;
	int nbFoundMarkers(0);

    // Convert Image from RGB to grey
    if (image->getNbChannels() != 1)
        m_imageConvertor->convert(image, greyImage, Image::ImageLayout::LAYOUT_GREY);
    else
        greyImage = image->copy();

	cv::Mat cvImage = SolAROpenCVHelper::mapToOpenCV(greyImage);
	std::vector<cv::Point2f> corners;
	std::vector<std::string> codes;
	std::vector<cv::Mat> rectifiedImage;
	m_qrDetector.detectAndDecodeMulti(cvImage, codes, corners, rectifiedImage);

	for (int i = 0; i < m_nbMarkers; ++i) {
		for (int j = 0; j < codes.size(); ++j)
			if (m_QRCodes[i]->getCode() == codes[j]) {
				// get 2D points
				std::vector<cv::Point2f> img2Dpts(corners.begin() + j * 4, corners.begin() + j * 4 + 4);
				for (const auto& pt : img2Dpts)
					pts2D.push_back(Point2Df(pt.x, pt.y));
				// get 3D corresponding points
				const std::vector<Point3Df>& pattern3DPts = m_pattern3DPoints[i];
				pts3D.insert(pts3D.end(), pattern3DPts.begin(), pattern3DPts.end());
				nbFoundMarkers++;
				break;
			}
	}

	LOG_DEBUG("Number of detected markers: {}", nbFoundMarkers);
	LOG_DEBUG("Number of corners: {}", pts2D.size());
	if (nbFoundMarkers == 0)
		return FrameworkReturnCode::_ERROR_;

	// Refine corner locations
	m_cornerRefinement->refine(greyImage, pts2D);

	// Compute the pose of the camera using a Perspective n Points algorithm using all corners of the detected markers
	if (m_pnp->estimate(pts2D, pts3D, pose) == FrameworkReturnCode::_SUCCESS)
	{
		std::vector<Point2Df> projected2DPts;
		m_projector->project(pts3D, projected2DPts, pose);
		float errorReproj(0.f);
		for (int j = 0; j < projected2DPts.size(); ++j)
			errorReproj += (projected2DPts[j] - pts2D[j]).norm();
		errorReproj /= projected2DPts.size();
		LOG_DEBUG("Mean reprojection error: {}", errorReproj);
		if (errorReproj < m_maxReprojError)
			return FrameworkReturnCode::_SUCCESS;
		pose = Transform3Df::Identity();
	}
	return FrameworkReturnCode::_ERROR_;
}

}
}
}
