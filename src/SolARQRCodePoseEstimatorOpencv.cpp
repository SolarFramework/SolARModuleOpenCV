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

#include "SolARQRCodePoseEstimatorOpencv.h"
#include "core/Log.h"
#include "SolAROpenCVHelper.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARQRCodePoseEstimatorOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARQRCodePoseEstimatorOpencv::SolARQRCodePoseEstimatorOpencv():ConfigurableBase(xpcf::toUUID<SolARQRCodePoseEstimatorOpencv>())
{
    addInterface<SolAR::api::solver::pose::ITrackablePose>(this);
    declareInjectable<SolAR::api::image::IImageConvertor>(m_imageConvertor);
    declareInjectable<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>(m_pnp);
    declareInjectable<SolAR::api::features::ICornerRefinement>(m_cornerRefinement);
    declareInjectable<SolAR::api::geom::IProject>(m_projector);
    declareProperty("maxReprojError", m_maxReprojError);
    LOG_DEBUG("SolARQRCodePoseEstimatorOpencv constructor");
}

FrameworkReturnCode SolARQRCodePoseEstimatorOpencv::setTrackable(const SRef<SolAR::datastructure::Trackable> trackable)
{
    if (trackable->getType() == TrackableType::QRCODE_MARKER)
    {
        m_QRCode = xpcf::utils::dynamic_pointer_cast<QRCode>(trackable);
        LOG_DEBUG("Decoding code: {}", m_QRCode->getCode());
		float width = m_QRCode->getWidth();
		float height = m_QRCode->getHeight();
        LOG_DEBUG("Size (width x height): {} x {}", width, height);
		// calculate 3D pattern points
		m_QRCode->getWorldCorners(m_pattern3DPoints);
    }
    else {
        LOG_ERROR("The SolARQRCodePoseEstimatorOpencv should only use a trackable of type QRCODE")
        return FrameworkReturnCode::_ERROR_;
    }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARQRCodePoseEstimatorOpencv::estimate(const SRef<SolAR::datastructure::Image> image,
															 const SolAR::datastructure::CameraParameters & camParams,
															 SolAR::datastructure::Transform3Df & pose)
{
    SRef<Image>				greyImage;    
    std::vector<Point2Df>	img2DPoints;

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

	int pos(-1);
	for (int i = 0; i < codes.size(); ++i)
		if (codes[i] == m_QRCode->getCode()) {
			pos = i;
			break;
		}
	if (pos == -1)
		return FrameworkReturnCode::_ERROR_;
	
	std::vector<cv::Point2f> pts(corners.begin() + pos * 4, corners.begin() + pos * 4 + 4);
	for (const auto& pt : pts)
		img2DPoints.push_back(Point2Df(pt.x, pt.y));

	// Refine corner locations
	m_cornerRefinement->refine(greyImage, img2DPoints);
    if (m_pnp->estimate(img2DPoints, m_pattern3DPoints, camParams, pose) == FrameworkReturnCode::_SUCCESS)
    {
        std::vector<Point2Df> projected2DPts;
        m_projector->project(m_pattern3DPoints, pose, camParams, projected2DPts);
        float errorReproj(0.f);
		// only calculate error repojection for confident corners
        for (int j = 0; j < projected2DPts.size(); ++j)
			if (j != 2)
				errorReproj += (projected2DPts[j] - img2DPoints[j]).norm();
        errorReproj /= projected2DPts.size();
        LOG_DEBUG("Mean reprojection error: {}", errorReproj);
        if (errorReproj < m_maxReprojError)
            return FrameworkReturnCode::_SUCCESS;        
    }
	pose = Transform3Df::Identity();
    return FrameworkReturnCode::_ERROR_;
}

}
}
}
