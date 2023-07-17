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
    declareInjectable<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>(m_pnp);
    declareInjectable<SolAR::api::features::I2DTrackablesDetector>(m_trackablesDetector);
    declareInjectable<SolAR::api::geom::IProject>(m_projector);
    declareProperty("maxReprojError", m_maxReprojError);
    LOG_DEBUG("SolARMultiQRCodesPoseEstimatorOpencv constructor");
}

FrameworkReturnCode SolARMultiQRCodesPoseEstimatorOpencv::setTrackables(const std::vector<SRef<SolAR::datastructure::Trackable>> trackables)
{
	m_nbMarkers = trackables.size();
	if (m_nbMarkers == 0)
		return FrameworkReturnCode::_ERROR_;
	    
    int nbFloatingTrackables(0);
    for (const auto &trackable : trackables)
        if (trackable->getType() == TrackableType::QRCODE_MARKER) {
			SRef<QRCode> qrCode = xpcf::utils::dynamic_pointer_cast<QRCode>(trackable);			
			// get 3D pattern points
			std::vector<Point3Df> pts3D;
            if (!qrCode->getTransform3D().matrix().isZero())
                qrCode->getWorldCorners(pts3D);
            else
                nbFloatingTrackables++;
			m_pattern3DPoints.push_back(pts3D);
        }
        else {
            LOG_ERROR("The SolARMultiQRCodesPoseEstimatorOpencv should only use a trackable of type QRCODE")
            return FrameworkReturnCode::_ERROR_;
        }
    LOG_DEBUG("Number of QR codes: {}", m_nbMarkers);
    LOG_DEBUG("Number of floating QR codes: {}", nbFloatingTrackables);
    // set trackables to detector
    m_trackablesDetector->setTrackables(trackables);
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMultiQRCodesPoseEstimatorOpencv::estimate(const SRef<SolAR::datastructure::Image> image,
                                                                   const SolAR::datastructure::CameraParameters & camParams,
                                                                   SolAR::datastructure::Transform3Df & pose)
{
    std::vector<Point2Df>	pts2D;
    std::vector<Point3Df>	pts3D;
	int nbFoundMarkers(0);

    //  detect qr codes
    std::vector<std::vector<Point2Df>> corners;
    m_trackablesDetector->detect(image, corners);

    if (corners.size() != m_pattern3DPoints.size())
        return FrameworkReturnCode::_ERROR_;

	for (int i = 0; i < m_nbMarkers; ++i) {
        if ((corners[i].size() > 0) && (m_pattern3DPoints[i].size() > 0)) {
            pts2D.insert(pts2D.end(), corners[i].begin(), corners[i].end());
            pts3D.insert(pts3D.end(), m_pattern3DPoints[i].begin(), m_pattern3DPoints[i].end());
            nbFoundMarkers++;
        }
	}
    LOG_DEBUG("Number of detected markers: {}", nbFoundMarkers);
	if (nbFoundMarkers == 0)
		return FrameworkReturnCode::_ERROR_;

	// Compute the pose of the camera using a Perspective n Points algorithm using all corners of the detected markers
	if (m_pnp->estimate(pts2D, pts3D, camParams, pose) == FrameworkReturnCode::_SUCCESS)
	{
		std::vector<Point2Df> projected2DPts;
		m_projector->project(pts3D, pose, camParams, projected2DPts);
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
