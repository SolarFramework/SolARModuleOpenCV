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
#include "ComponentBase.h"
#include "SolAROpencvAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARDescriptorsExtractorSBPatternOpencv : public org::bcom::xpcf::ComponentBase,
        public api::features::IDescriptorsExtractorSBPattern {
public:
    SolARDescriptorsExtractorSBPatternOpencv();
    ~SolARDescriptorsExtractorSBPatternOpencv() = default;

    void setParameters (const int patternSize) override;

    FrameworkReturnCode extract(const std::vector<SRef<Image>>& inputImages, const std::vector<SRef<Contour2Df>>& contours, SRef<DescriptorBuffer> & pattern_descriptors, std::vector<SRef<Contour2Df>> & recognized_contours) override;
    FrameworkReturnCode extract(const SRef<SquaredBinaryPattern> pattern, SRef<DescriptorBuffer> & descriptor) override;

    void unloadComponent () override final;
    XPCF_DECLARE_UUID("d25625ba-ce3a-11e7-abc4-cec278b6b50a");

private:
    FrameworkReturnCode getPatternDescriptorFromImage (SRef<Image> image, unsigned char* data);
    bool isPattern(SRef<Image> image);

private:
    // Define the internal size of the pattern (without the black border), with first the number of rows, and secondly the number of columns
    int m_patternSize;
};

}
}
}  // end of namespace Solar

#endif // SOLARDESCRIPTORSEXTRACTORSBPATTERNOPENCV_H
