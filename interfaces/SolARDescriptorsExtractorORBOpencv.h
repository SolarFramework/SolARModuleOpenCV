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

#ifndef SOLARDESCRIPTORSEXTRACTOORBROPENCV_H
#define SOLARDESCRIPTORSEXTRACTOORBROPENCV_H

#include "api/features/IDescriptorsExtractor.h"

// Definition of SolARDescriptorExtractorOpencv Class //
// part of SolAR namespace //

#include "xpcf/component/ComponentBase.h"
#include "SolAROpencvAPI.h"
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "datastructure/DescriptorBuffer.h"
#include "datastructure/Keypoint.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARDescriptorsExtractorORBOpencv
 * @brief <B>Extracts the ORB descriptors for a set of keypoints.</B>
 * <TT>UUID: 0ca8f7a6-d0a7-11e7-8fab-cec278b6b50a</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARDescriptorsExtractorORBOpencv : public org::bcom::xpcf::ComponentBase,
        public api::features::IDescriptorsExtractor {
public:
    SolARDescriptorsExtractorORBOpencv();
    ~SolARDescriptorsExtractorORBOpencv() override;
    void unloadComponent () final;
    std::string getTypeString() override { return std::string("DescriptorsExtractorType::ORB"); }
    /// @brief Extracts a set of descriptors from a given image around a set of keypoints based on ORB algorithm
    /// "ORB: an efficient alternative to SIFT or SURF"
    /// [in] image: source image.
    /// [in] keypoints: set of keypoints.
    /// [out] decsriptors: se of computed descriptors.
    void extract (const SRef<Image> image,
                  const std::vector< Keypoint > &keypoints,
                  SRef<DescriptorBuffer> & descriptors) override;
private:
    cv::Ptr<cv::Feature2D> m_extractor;
};

}
}
}  // end of namespace SolAR



#endif // SOLARDESCRIPTORSEXTRACTOORBROPENCV_H
