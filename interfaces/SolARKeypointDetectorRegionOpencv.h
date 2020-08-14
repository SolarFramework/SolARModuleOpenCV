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

#ifndef SOLARKEYPOINTDETECTORREGIONOPENCV_H
#define SOLARKEYPOINTDETECTORREGIONOPENCV_H

#include "api/features/IKeypointDetectorRegion.h"

// Definition of SolARKeypointDetectorOpencv Class //
// part of SolAR namespace //

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include <string>
#include "opencv2/opencv.hpp"
#if ((CV_VERSION_MAJOR < 4 ) || (CV_VERSION_MINOR < 4 ))
    #include "opencv2/xfeatures2d.hpp" // Define SIFT
#endif
#include "features2d_akaze2.hpp"  // Define AKAZE2;

namespace SolAR {
using namespace datastructure;
using namespace api::features;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARKeypointDetectorRegionOpencv
 * @brief <B>Detects keypoints in an given region of an image.</B>
 * <TT>UUID: 22c2ca9f-e43b-4a88-8337-4a166a789971</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARKeypointDetectorRegionOpencv : public org::bcom::xpcf::ConfigurableBase,
        public IKeypointDetectorRegion {
public:

    /// @brief SolARKeypointDetectorRegionOpencv default constructor
    SolARKeypointDetectorRegionOpencv();

    /// @brief SolARKeypointDetectorRegionOpencv default destructor
    ~SolARKeypointDetectorRegionOpencv() override;

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

    /// @brief Set the type of method used to detect keypoints in the image
    /// @param[in] type The type of method used to detect keypoints.
    void setType(IKeypointDetector::KeypointDetectorType type) override;

    /// @brief Get the type of method used to detect keypoints in the image
    /// @return The type of method used to detect keypoints.
    IKeypointDetector::KeypointDetectorType  getType() override;
 
    /// @brief This method detects keypoints in an input Image
    /// @param[in] image input image on which we are extracting keypoints.
    /// @param[in] contours a set of 2D points defining the contour of the region where keypoints will be detected
    /// @param[out] keypoints The keypoints detected from the given region of the image passed as first argument.
    void detect (const SRef<Image> image,
                             const std::vector<Point2Df> & contours,
                             std::vector<Keypoint> & keypoints) override;

    void unloadComponent () override final;

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



#endif // SOLARKEYPOINTDETECTORREGIONOPENCV_H
