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

#ifndef SOLAROPTICALFLOWPYRLKOPENCV_H
#define SOLAROPTICALFLOWPYRLKOPENCV_H

#include "api/tracking/IOpticalFlowEstimator.h"

// Definition of SolAROpticalFlowPyrLKOpencv Class //
// part of SolAR namespace //

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include <string>
#include "opencv2/video/tracking.hpp""

#include "datastructure/Image.h"
#include "datastructure/Keypoint.h"

namespace SolAR {
using namespace datastructure;
using namespace api::tracking;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolAROpticalFlowPyrLKOpencv
 * @brief <B>Estimates the optical flow between two images based on a pyramidal Lucas Kanade approach.</B>
 * <TT>UUID: e95302be-3fe1-44e0-97bf-a98380464af9</TT>
 *
 */


class SOLAROPENCV_EXPORT_API SolAROpticalFlowPyrLKOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::tracking::IOpticalFlowEstimator {
public:
    SolAROpticalFlowPyrLKOpencv();
    ~SolAROpticalFlowPyrLKOpencv();
    void unloadComponent () override final;

    /// @brief estimate the optical flow between two images
    /// @param[in] previousImage The previous image
    /// @param[in] currentImage The current image for which we want to estimate the optical flow relative to the previous image
    /// @param[in] pointsToTrack The pixels to track in the previous image
    /// @param[out] trackedPoints The position of the pointsToTrack in the current image
    /// @param[out] status Specify for each point; each element of the vector is set to 1 if the flow for the corresponding features has been found, otherwise, it is set to 0.
    /// @param[out] error Specify for each point the tracking error
    FrameworkReturnCode estimate(const SRef<Image> previousImage,
                                 const SRef<Image> currentImage,
                                 const std::vector<SRef<Keypoint>> & pointsToTrack,
                                 std::vector<SRef<Point2Df>> & trackedPoints,
                                 std::vector<unsigned char> & status,
                                 std::vector<float> & error);

    /// @brief estimate the optical flow between two images
    /// @param[in] previousImage The previous image
    /// @param[in] currentImage The current image for which we want to estimate the optical flow relative to the previous image
    /// @param[in] pointsToTrack The pixels to track in the previous image
    /// @param[out] trackedPoints The position of the pointsToTrack in the current image
    /// @param[out] status Specify for each point; each element of the vector is set to 1 if the flow for the corresponding features has been found, otherwise, it is set to 0.
    /// @param[out] error Specify for each point the tracking error
    FrameworkReturnCode estimate(const SRef<Image> previousImage,
                                 const SRef<Image> currentImage,
                                 const std::vector<SRef<Point2Df>> & pointsToTrack,
                                 std::vector<SRef<Point2Df>> & trackedPoints,
                                 std::vector<unsigned char> & status,
                                 std::vector<float> & error);

private:

    /// @brief WidthHeight in pixels of the search window at each pyramid level
    int m_searchWinWidth = 21;

    /// @brief Height in pixels of the search window at each pyramid level
    int m_searchWinHeight = 21;

    /// @brief maximum level of pyramid
    /// If set to 0, pyramids are not used (single level), if set to 1, two levels are used, and so on; if pyramids are passed to input then algorithm will use as many levels as pyramids have but no more than maxLevel.
    int m_maxLevel = 3;

    /// The minimum eigen value to measure error
    /// The algorithm calculates the minimum eigen value of a 2x2 normal matrix of optical flow equations (this matrix is called a spatial gradient matrix in [16]), divided by number of pixels in a window.
    /// If this value is less than minEigThreshold, then a corresponding feature is filtered out and its flow is not processed, so it allows to remove bad points and get a performance boost.
    /// If this value is less or equal to 0, the minimum eigen values as an error measure are not used. Instead, the L1 distance between patches around the original and a moved point, divided by number of pixels in a window, is used as a error measure.
    double m_minEigenThreshold = -1.0;

    // The maximum iteration before iterative search algorithm stops.
    int m_maxSearchIterations = 20;

    // The desired accuracy of the search window before algorithm stops.
    float m_searchWindowAccuracy = 0.03;


    FrameworkReturnCode estimate(const SRef<Image> previousImage,
                                 const SRef<Image> currentImage,
                                 const std::vector<cv::Point2f> & pointsToTrack,
                                 std::vector<SRef<Point2Df>> & trackedPoints,
                                 std::vector<unsigned char> & status,
                                 std::vector<float> & error);
};

}
}
}  // end of namespace SolAR

#endif // SOLAROPTICALFLOWPYRLKOPENCV_H
