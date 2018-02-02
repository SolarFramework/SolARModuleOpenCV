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

#ifndef SOLARDESCRIPTORSEXTRACTOSIFTROPENCV_H
#define SOLARDESCRIPTORSEXTRACTOSIFTROPENCV_H

#include "api/features/IDescriptorsExtractor.h"
// Definition of SolARDescriptorExtractorOpencv Class //
// part of SolAR namespace //

#include "ComponentBase.h"
#include "SolAROpencvAPI.h"
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"

namespace SolAR {
using namespace datastructure;

namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARDescriptorsExtractorSIFTOpencv : public org::bcom::xpcf::ComponentBase,
        public api::features::IDescriptorsExtractor {
public:
    SolARDescriptorsExtractorSIFTOpencv();
    ~SolARDescriptorsExtractorSIFTOpencv();
    void unloadComponent () override final;
    inline std::string getTypeString() override { return std::string("DescriptorExtractorType::SIFT") ;};

    void extract (const SRef<Image> image, const std::vector<SRef<Keypoint> > &keypoints, SRef<DescriptorBuffer>& descriptors) override;

    XPCF_DECLARE_UUID("3787eaa6-d0a0-11e7-8fab-cec278b6b50a");

private:
    cv::Ptr<cv::Feature2D> m_extractor;
};

}
}
}  // end of namespace SolAR



#endif // SOLARDESCRIPTORSEXTRACTOSIFTROPENCV_H
