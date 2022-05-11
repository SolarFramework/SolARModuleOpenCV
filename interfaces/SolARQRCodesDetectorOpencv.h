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

#ifndef SOLARQRCODESDETECTOROPENCV_H
#define SOLARQRCODESDETECTOROPENCV_H
#include "api/features/I2DTrackablesDetector.h"
#include "api/features/ICornerRefinement.h"
#include "api/image/IImageConvertor.h"
#include "datastructure/QRCode.h"
#include "SolAROpencvAPI.h"
#include "xpcf/component/ConfigurableBase.h"
#include "opencv2/opencv.hpp"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARQRCodesDetectorOpencv
* @brief <B>Detect a set of given 2D trackables in an image.</B>
* <TT>UUID: 0ff5ae31-f469-4d9b-86fc-feca1fa74a04</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::image::IImageConvertor}
* @SolARComponentInjectable{SolAR::api::features::ICornerRefinement}
* @SolARComponentInjectablesEnd
*
*/

class SOLAROPENCV_EXPORT_API SolARQRCodesDetectorOpencv : public org::bcom::xpcf::ConfigurableBase,
    public SolAR::api::features::I2DTrackablesDetector
{
public:
    ///@brief SolARQRCodesDetectorOpencv constructor;
    SolARQRCodesDetectorOpencv();

    ///@brief SolARQRCodesDetectorOpencv destructor;
    ~SolARQRCodesDetectorOpencv() = default;

    /// @brief this method is used to set the set of 2D trackables.
    /// @param[in] trackables the set of 2D trackables.
    FrameworkReturnCode setTrackables(const std::vector<SRef<SolAR::datastructure::Trackable>> trackables) override;

    /// @brief Detect a set of trackables.
    /// @param[in] image input image.
    /// @param[out] corners a set of detected corners corresponding to the trackables (each trackable has a set of 4 corners).
    /// @return FrameworkReturnCode::_SUCCESS if the detection succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode detect(const SRef<SolAR::datastructure::Image> image,
                               std::vector<std::vector<SolAR::datastructure::Point2Df>> & corners) override;

    void unloadComponent() override final;

private:
    std::vector<SRef<SolAR::datastructure::QRCode>>             m_QRCodes;
    SRef<SolAR::api::image::IImageConvertor>					m_imageConvertor;
    SRef<SolAR::api::features::ICornerRefinement>               m_cornerRefinement;
	cv::QRCodeDetector											m_qrDetector;
	int															m_nbMarkers;
};

}
}
}

#endif // SOLARQRCODESDETECTOROPENCV_H
