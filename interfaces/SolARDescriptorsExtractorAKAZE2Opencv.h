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

#ifndef SOLARDESCRIPTORSEXTRACTORAKAZE2OPENCV_H
#define SOLARDESCRIPTORSEXTRACTORAKAZE2OPENCV_H

#include "api/features/IDescriptorsExtractor.h"
#include "SolARImageConvertorOpencv.h"

// Definition of SolARDescriptorExtractorOpencv Class //
// part of SolAR namespace //

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include <string>
#include "opencv2/opencv.hpp"
#include "features2d_akaze2.hpp"  // Define AKAZE2;
#include "datastructure/DescriptorBuffer.h"
#include "datastructure/Keypoint.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARDescriptorsExtractorAKAZE2Opencv
 * @brief <B>Extracts the AKAZE descriptors for a set of keypoints (optimized version).</B>
 * <TT>UUID: 21238c00-26dd-11e8-b467-0ed5f89f718b</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARDescriptorsExtractorAKAZE2Opencv : public org::bcom::xpcf::ConfigurableBase,
        public api::features::IDescriptorsExtractor {
public:
    SolARDescriptorsExtractorAKAZE2Opencv();
    ~SolARDescriptorsExtractorAKAZE2Opencv() override;

    org::bcom::xpcf::XPCFErrorCode onConfigured() final;
    void unloadComponent () final;
    inline std::string getTypeString() override { return std::string("DescriptorsExtractorType::AKAZE2") ;};
    /// @brief Extracts a set of descriptors from a given image around a set of keypoints based on AKAZE 2 algorithm
    /// "Fast explicit diffusion for acceleratedfeatures in nonlinear scale space"
    /// [in] image: source image.
    /// [in] keypoints: set of keypoints.
    /// [out] decsriptors: se of computed descriptors.
    void extract (const SRef<Image> image,
                  const std::vector< Keypoint > &keypoints,
                  SRef<DescriptorBuffer> & descriptors) override;
private:
    cv::Ptr<cv::AKAZE2> m_extractor;
    SolARImageConvertorOpencv m_convertor;

    double m_threshold = 3e-4;
};

}
}
}  // end of namespace SolAR



#endif // SOLARDESCRIPTORSEXTRACTORAKAZE2OPENCV_H
