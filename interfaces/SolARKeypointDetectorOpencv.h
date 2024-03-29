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
#if ((CV_VERSION_MAJOR < 4 ) || (CV_VERSION_MINOR < 4 ))
    #include "opencv2/xfeatures2d.hpp" // Define SIFT
#endif
#include "features2d_akaze2.hpp"  // Define AKAZE2;



namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARKeypointDetectorOpencv
 * @brief <B>Detects keypoints in an image.</B>
 * <TT>UUID: e81c7e4e-7da6-476a-8eba-078b43071272</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ imageRatio,
 *                          the ratio to apply to the size of the input image to compute the descriptor.<br>
 *                               A ratio must be less or equal to 1. A ratio less than 1 will speedup computation,
 *                          @SolARComponentPropertyDescNum{ float, [0..1], 1.f }}
 * @SolARComponentProperty{ nbDescriptors,
 *                          the number of descriptors that are selected. If negative\, all extracted descriptors are selected,
 *                          @SolARComponentPropertyDescNum{ int, [-1..MAX INT], 1000 }}
 * @SolARComponentProperty{ threshold,
 *                          the threshold of detector to accept a keypoint,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 1e-3f }}
 * @SolARComponentProperty{ nbOctaves,
 *                          the number of octaves,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 4 }}
 * @SolARComponentProperty{ type,
 *                          type of descriptor used for the extraction (SIFT\, AKAZE\, AKAZE2\, ORB\, BRISK),
 *                          @SolARComponentPropertyDescString{ "AKAZE2" }}
 * @SolARComponentPropertiesEnd
 */

class SOLAROPENCV_EXPORT_API SolARKeypointDetectorOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::features::IKeypointDetector {
public:

    SolARKeypointDetectorOpencv();
    ~SolARKeypointDetectorOpencv() override;
    void unloadComponent () override final;

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

    /// @brief Set the type of method used to detect keypoints in the image
    /// @param[in] type The type of method used to detect keypoints.
    void setType(KeypointDetectorType type) override;

    /// @brief Get the type of method used to detect keypoints in the image
    /// @return The type of method used to detect keypoints.
    KeypointDetectorType  getType() override;
 
    /// @brief This method detects keypoints in an input Image
    /// @param[in] image input image on which we are extracting keypoints.
    /// @param[out] keypoints The keypoints detected from the image passed as first argument.
    void detect (const SRef<datastructure::Image> image, std::vector<datastructure::Keypoint> & keypoints) override;

private:
    /// @brief the type of descriptor used for the extraction (SIFT, AKAZE, AKAZE2, ORB, BRISK)
    std::string m_type = "AKAZE2";

    /// @brief the ratio to apply to the size of the input image to compute the descriptor.
    /// A ratio must be less or equal to 1. A ratio less than 1 will speedup computation
    float m_imageRatio=1.0f;

    /// @brief the number of descriptors that are selected. If negative, all extracted descriptors are selected
    int m_nbDescriptors = 10000;

	/// @brief the number of octaves
	int m_nbOctaves = 4;

	/// @brief the threshold of detector to accept a keypoint
    float m_threshold = 1e-3f;

    int m_id;
    cv::Ptr<cv::Feature2D> m_detector;
    cv::KeyPointsFilter m_kptsFilter;
	int m_nbGridWidth = 20;
	int m_nbGridHeight = 20;
	float m_borderRatio = 0.01f;

};

extern int deduceOpenCVType(SRef<datastructure::Image> img);

}
}
}  // end of namespace SolAR



#endif // SOLARKEYPOINTDETECTOROPENCV_H
