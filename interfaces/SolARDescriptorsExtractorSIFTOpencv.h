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

#include "SolAROpencvAPI.h"
#include <string>

//opencv headers
#include "opencv2/opencv.hpp"
#if ((CV_VERSION_MAJOR < 4 ) || (CV_VERSION_MINOR < 4 ))
    #include "opencv2/xfeatures2d.hpp"
#endif

//solar headers
#include "api/features/IDescriptorsExtractor.h"

//xpcf headers
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
using namespace datastructure;

namespace MODULES {
namespace OPENCV {

/**
 * @class SolARDescriptorsExtractorSIFTOpencv
 * @brief <B>Extracts the SIFT descriptors for a set of keypoints.</B>
 * <TT>UUID: 3787eaa6-d0a0-11e7-8fab-cec278b6b50a</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARDescriptorsExtractorSIFTOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::features::IDescriptorsExtractor {
public:
    SolARDescriptorsExtractorSIFTOpencv();
    ~SolARDescriptorsExtractorSIFTOpencv();
    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

    void unloadComponent () override final;
    inline std::string getTypeString() override { return std::string("DescriptorExtractorType::SIFT") ;};

    /// @brief Extracts a set of descriptors from a given image around a set of keypoints based on SIFT algorithm
    /// [in] image: source image.
    /// [in] keypoints: set of keypoints.
    /// [out] decsriptors: set of computed descriptors.
    void extract (const SRef<Image> image, const std::vector<Keypoint> & keypoints, SRef<DescriptorBuffer>& descriptors) override;

private:
    cv::Ptr<cv::Feature2D> m_extractor;
    int m_nbFeatures = 0;
    int m_nbOctaveLayers = 3;
    double m_contrastThreshold = 0.04;
    double m_edgeThreshold = 10.0;
    double m_sigma = 1.6;
};

}
}
}  // end of namespace SolAR



#endif // SOLARDESCRIPTORSEXTRACTOSIFTROPENCV_H
