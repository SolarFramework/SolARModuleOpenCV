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

#ifndef SOLARKEYPOINTDETECTOROPENCV_H
#define SOLARKEYPOINTDETECTOROPENCV_H

#include "api/features/IKeypointDetector.h"

// Definition of SolARKeypointDetectorOpencv Class //
// part of SolAR namespace //

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include <string>
#include "opencv2/opencv.hpp"
#include "features2d_akaze2.hpp"  // Define AKAZE2;



namespace SolAR {
using namespace datastructure;
using namespace api::features;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARKeypointDetectorOpencv : public org::bcom::xpcf::ConfigurableBase,
        public IKeypointDetector {
public:

    SolARKeypointDetectorOpencv();
    ~SolARKeypointDetectorOpencv();
    void unloadComponent () override final;

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

    void setType(KeypointDetectorType type);
    KeypointDetectorType  getType();
 
    void detect (const SRef<Image> &image, std::vector<SRef<Keypoint>> &keypoints);

private:
    /// @brief the type of descriptor used for the extraction (AKAZE, AKAZE2, ORB, BRISK)
    std::string m_type = "AKAZE2";

    /// @brief the ratio to apply to the size of the input image to compute the descriptor.
    /// A ratio must be less or equal to 1. A ratio less than 1 will speedup computation
    float m_imageRatio=1.0f;

    /// @brief the number of descriptors that are selected. If negative, all extracted descriptors are selected
    int m_nbDescriptors = 10000;

	/// @brief the threshold of detector to accept a keypoint
	float m_threshold = 1e-3;

    int m_id;
    cv::Ptr<cv::Feature2D> m_detector;
    cv::KeyPointsFilter kptsFilter;

};

extern int deduceOpenCVType(SRef<Image> img);

}
}
}  // end of namespace SolAR



#endif // SOLARKEYPOINTDETECTOROPENCV_H
