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

#ifndef SOLARMULTIQRCODESPOSEESTIMATOROPENCV_H
#define SOLARMULTIQRCODESPOSEESTIMATOROPENCV_H
#include "api/solver/pose/IMultiTrackablesPose.h"
#include "api/features/I2DTrackablesDetector.h"
#include "api/geom/IProject.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "datastructure/QRCode.h"
#include "SolAROpencvAPI.h"
#include "xpcf/component/ConfigurableBase.h"
#include "opencv2/opencv.hpp"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARMultiQRCodesPoseEstimatorOpencv
* @brief <B>Estimate camera pose based on a set of QR codes.</B>
* <TT>UUID: 73e66f7f-be35-4d76-97f2-ef864e043d57</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::features::IMultiTrackablesPose}
* @SolARComponentInjectable{SolAR::api::solver::pose::I3DTransformFinderFrom2D3D}
* * @SolARComponentInjectable{SolAR::api::geom::IProject}
* @SolARComponentInjectablesEnd
*
* @SolARComponentPropertiesBegin
* @SolARComponentProperty{ maxReprojError,
*                          ,
*                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 1.0 }}
* @SolARComponentPropertiesEnd
*
*/

class SOLAROPENCV_EXPORT_API SolARMultiQRCodesPoseEstimatorOpencv : public org::bcom::xpcf::ConfigurableBase,
    public SolAR::api::solver::pose::IMultiTrackablesPose
{
public:
    ///@brief SolARMultiQRCodesPoseEstimatorOpencv constructor;
    SolARMultiQRCodesPoseEstimatorOpencv();

    ///@brief SolARMultiQRCodesPoseEstimatorOpencv destructor;
    ~SolARMultiQRCodesPoseEstimatorOpencv() = default;

    /// @brief this method is used to set intrinsic parameters and distorsion of the camera
    /// @param[in] Camera calibration matrix parameters.
    /// @param[in] Camera distorsion parameters.
    void setCameraParameters(const SolAR::datastructure::CamCalibration & intrinsicParams, const SolAR::datastructure::CamDistortion & distorsionParams) override;;

	/// @brief this method is used to set the trackables used to estimate the pose.
	/// @param[in] trackables the set of trackables used to estimate the pose.
	FrameworkReturnCode setTrackables(const std::vector<SRef<SolAR::datastructure::Trackable>> trackables) override;

	/// @brief Estimates camera pose based on a set of QR codes.
	/// @param[in] image input image.
	/// @param[out] pose camera pose.
	/// @return FrameworkReturnCode::_SUCCESS if the estimation succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode estimate(const SRef<SolAR::datastructure::Image> image, SolAR::datastructure::Transform3Df & pose) override;

    void unloadComponent() override final;

private:
    SolAR::datastructure::CamCalibration						m_camMatrix;
    SolAR::datastructure::CamDistortion                         m_camDistortion;
    std::vector<SRef<SolAR::datastructure::QRCode>>             m_QRCodes;
    SRef<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>	m_pnp;
    SRef<SolAR::api::features::I2DTrackablesDetector>           m_trackablesDetector;
    SRef<SolAR::api::geom::IProject>							m_projector;
    float                                                       m_maxReprojError = 1.0f;
	std::vector<std::vector<SolAR::datastructure::Point3Df>>	m_pattern3DPoints;
	int															m_nbMarkers;
};

}
}
}

#endif // SOLARMULTIQRCODESPOSEESTIMATOROPENCV_H
