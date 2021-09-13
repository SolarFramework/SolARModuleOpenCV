/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
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

#ifndef SOLARDESCRIPTORSEXTRACTORFROMIMAGEOPENCV_H
#define SOLARDESCRIPTORSEXTRACTORFROMIMAGEOPENCV_H
#include "api/features/IDescriptorsExtractorFromImage.h"
#include "SolAROpencvAPI.h"
#include "xpcf/component/ConfigurableBase.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARDescriptorsExtractorFromImageOpencv
 * @brief <B>Detect keypoints and compute the descriptors from an image.</B>
 * <TT>UUID: cf2721f2-0dc9-4442-ad1e-90c0ab12b0ff</TT>
 *
 * @SolARComponentInjectablesBegin
 * @SolARComponentInjectable{SolAR::api::features::IKeypointDetector}
 * @SolARComponentInjectable{SolAR::api::features::IDescriptorsExtractor}
 * @SolARComponentInjectablesEnd
 *
 */

class SOLAROPENCV_EXPORT_API SolARDescriptorsExtractorFromImageOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::features::IDescriptorsExtractorFromImage
{
public:
    ///@brief SolARDescriptorsExtractorFromImageOpencv constructor;
    SolARDescriptorsExtractorFromImageOpencv();
    ///@brief SolARDescriptorsExtractorFromImageOpencv destructor;
    ~SolARDescriptorsExtractorFromImageOpencv();

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

    /// @brief getType
    /// @return a string describing the type of descriptor used during extraction.
    std::string getTypeString() override;

    /// @brief detect keypoints and compute the descriptors.
    /// @param[in] image image on which the keypoint and their descriptor will be detected and extracted.
    /// @param[out] keypoints The keypoints detected in the input image.
    /// @param[out] descriptors The descriptors of keypoints of the input image.
    /// @return FrameworkReturnCode::_SUCCESS_ if images are well matched, else FrameworkReturnCode::_ERROR
    FrameworkReturnCode extract(const SRef<SolAR::datastructure::Image> image,
                                std::vector<SolAR::datastructure::Keypoint> &keypoints,
                                SRef<SolAR::datastructure::DescriptorBuffer> & descriptors) override;

    void unloadComponent () override final;

private:
    SRef<SolAR::api::features::IKeypointDetector>       m_detector;
    SRef<SolAR::api::features::IDescriptorsExtractor>   m_extractor;
};

}
}
}


#endif // SOLARDESCRIPTORSEXTRACTORFROMIMAGEOPENCV_H
