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

#ifndef SOLARDESCRIPTORSEXTRACTORAKAZEOPENCV_H
#define SOLARDESCRIPTORSEXTRACTORAKAZEOPENCV_H

#include "api/features/IDescriptorsExtractor.h"

// Definition of SolARDescriptorExtractorOpencv Class //
// part of SolAR namespace //

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "datastructure/DescriptorBuffer.h"
#include "datastructure/Keypoint.h"
#include "SolARImageConvertorOpencv.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARDescriptorsExtractorAKAZEOpencv
 * @brief <B>Extracts the AKAZE descriptors for a set of keypoints.</B>
 * <TT>UUID: c8cc68db-9abd-4dab-9204-2fe4e9d010cd</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ threshold,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ double, [0..MAX DOUBLE], 3e-4 }}
 * @SolARComponentPropertiesEnd
 */

class SOLAROPENCV_EXPORT_API SolARDescriptorsExtractorAKAZEOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::features::IDescriptorsExtractor {
public:
    SolARDescriptorsExtractorAKAZEOpencv();
    ~SolARDescriptorsExtractorAKAZEOpencv();

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;
    void unloadComponent () override final;
    inline std::string getTypeString() override { return std::string("DescriptorsExtractorType::AKAZE") ;};

    /// @brief Extracts a set of descriptors from a given image around a set of keypoints based on AKAZE algorithm
    /// "Fast explicit diffusion for acceleratedfeatures in nonlinear scale space"
    /// [in] image: source image.
    /// [in] keypoints: set of keypoints.
    /// [out] decsriptors: se of computed descriptors.
    void extract (const SRef<datastructure::Image> image,
                  const std::vector< datastructure::Keypoint > &keypoints,
                  SRef<datastructure::DescriptorBuffer> & descriptors) override;

private:
    cv::Ptr<cv::AKAZE> m_extractor;
    double m_threshold = 3e-4;
	SolARImageConvertorOpencv m_convertor;
};

}
}
}  // end of namespace SolAR



#endif // SOLARDESCRIPTORSEXTRACTORAKAZEOPENCV_H
