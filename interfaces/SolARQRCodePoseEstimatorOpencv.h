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

#ifndef SOLARQRCODEPOSEESTIMATOROPENCV_H
#define SOLARQRCODEPOSEESTIMATOROPENCV_H
#include "api/solver/pose/ITrackablePose.h"
#include "api/image/IImageConvertor.h"
#include "api/geom/IProject.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "api/features/ICornerRefinement.h"
#include "datastructure/Image.h"
#include "datastructure/QRCode.h"
#include "SolAROpencvAPI.h"
#include "xpcf/component/ConfigurableBase.h"
#include "opencv2/opencv.hpp"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARQRCodePoseEstimatorOpencv
* @brief <B>Estimate camera pose based on a QR code.</B>
* <TT>UUID: 7cd9c6ea-9287-4058-9e18-c64129c017c8</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::image::IImageConvertor}
* @SolARComponentInjectable{SolAR::api::solver::pose::I3DTransformFinderFrom2D3D}
* @SolARComponentInjectable{SolAR::api::features::ICornerRefinement}
* * @SolARComponentInjectable{SolAR::api::geom::IProject}
* @SolARComponentInjectablesEnd
*
* @SolARComponentPropertiesBegin
* @SolARComponentProperty{ maxReprojError,
*                          ,
*                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0.5 }}
* @SolARComponentPropertiesEnd
*
*/

class SOLAROPENCV_EXPORT_API SolARQRCodePoseEstimatorOpencv : public org::bcom::xpcf::ConfigurableBase,
    public SolAR::api::solver::pose::ITrackablePose
{
public:
    ///@brief SolARQRCodePoseEstimatorOpencv constructor;
    SolARQRCodePoseEstimatorOpencv();

    ///@brief SolARQRCodePoseEstimatorOpencv destructor;
    ~SolARQRCodePoseEstimatorOpencv() = default;

    /// @brief this method is used to set the trackable used to estimate the pose.
    /// @param[in] the trackable used to estimate the pose.
    FrameworkReturnCode setTrackable(const SRef<SolAR::datastructure::Trackable> trackable) override;

    /// @brief Estimates camera pose based on a fiducial marker.
    /// @param[in] image input image.
    /// @param[in] camParams the camera parameters.
    /// @param[out] pose camera pose.
    /// @return FrameworkReturnCode::_SUCCESS if the estimation succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode estimate(const SRef<SolAR::datastructure::Image> image,
                                 const SolAR::datastructure::CameraParameters & camParams,
                                 SolAR::datastructure::Transform3Df & pose) override;

    void unloadComponent() override final;

private:
    SRef<SolAR::datastructure::QRCode>                          m_QRCode;
    SRef<SolAR::api::image::IImageConvertor>					m_imageConvertor;
    SRef<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>	m_pnp;
    SRef<SolAR::api::features::ICornerRefinement>				m_cornerRefinement;
    SRef<SolAR::api::geom::IProject>							m_projector;
    float                                                       m_maxReprojError = 0.5f;
	cv::QRCodeDetector											m_qrDetector;
	std::vector<SolAR::datastructure::Point3Df>					m_pattern3DPoints;
};

}
}
}

#endif // SOLARQRCODEPOSEESTIMATOROPENCV_H
