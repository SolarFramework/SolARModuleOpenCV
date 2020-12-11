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

#ifndef SOLARDESCRIPTORSEXTRACTORSBPATTERNOPENCV_H
#define SOLARDESCRIPTORSEXTRACTORSBPATTERNOPENCV_H

#include "api/features/IDescriptorsExtractorSBPattern.h"

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARDescriptorsExtractorSBPatternOpencv
 * @brief <B>Extracts the descriptor corresponding to a squared binary marker pattern.</B>
 * <TT>UUID: d25625ba-ce3a-11e7-abc4-cec278b6b50a</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARDescriptorsExtractorSBPatternOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::features::IDescriptorsExtractorSBPattern {
public:
    SolARDescriptorsExtractorSBPatternOpencv();
    ~SolARDescriptorsExtractorSBPatternOpencv() override = default;
    /// @brief Extracts a set of descriptors from a given images based on a marker pattern
    /// [in] : set of keypoints.
    /// [out] decsriptors: se of computed descriptors.
    FrameworkReturnCode extract(const std::vector<SRef<datastructure::Image>> & images,
                                           const std::vector<datastructure::Contour2Df> & inContours,
                                           SRef<datastructure::DescriptorBuffer> & descriptors,
                                           std::vector<datastructure::Contour2Df> & outContours) override;

    FrameworkReturnCode extract(const datastructure::SquaredBinaryPattern & pattern,
                                           SRef<datastructure::DescriptorBuffer> & descriptor) override;

    void unloadComponent () override final;

private:
    FrameworkReturnCode getPatternDescriptorFromImage (SRef<datastructure::Image> image, unsigned char* data);
    bool isPattern(SRef<datastructure::Image> image);

private:
    // Define the internal size of the pattern (without the black border)
    int m_patternSize = 5;
};

}
}
}  // end of namespace Solar

#endif // SOLARDESCRIPTORSEXTRACTORSBPATTERNOPENCV_H
