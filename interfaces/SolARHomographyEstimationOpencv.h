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

#ifndef SolARHomographyEstimationOpencv_H
#define SolARHomographyEstimationOpencv_H
#include <vector>

#include "api/solver/pose/I2DTransformFinder.h"

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include <vector>
#include "opencv2/core.hpp"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARHomographyEstimationOpencv
 * @brief <B>Estimates the homography between two images from their matching keypoints.</B>
 * <TT>UUID: fb9dac20-2a44-44b2-aa42-2871eec31427</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARHomographyEstimationOpencv : public org::bcom::xpcf::ConfigurableBase,
    public api::solver::pose::I2DTransformFinder {

public:
    SolARHomographyEstimationOpencv();
    ~SolARHomographyEstimationOpencv() override = default;

    /// @brief Computes an homography transformation from a set of source points and target points.
    /// [in] srcPoints: set of source points.
    /// [in] targetPints: set of target points.
    /// [out] homography: 3x3  homography matrice transformation.
    /// @return Transform2DFinder::RetCode::TRANSFORM2D_ESTIMATION_OK if succed.
    api::solver::pose::Transform2DFinder::RetCode find(const std::vector<Point2Df> & srcPoints,
                                                      const std::vector<Point2Df> & targetPoints,
                                                      Transform2Df & homography) override;

    void unloadComponent () final;

private:
    bool isHValid(const Transform2Df & H);
    float computeSurface(std::vector<cv::Point2f> points);
    float computeDistance(cv::Point2f a, cv::Point2f b);

    std::vector<cv::Point2f> obj_corners;
    std::vector<cv::Point2f> scene_corners;

    int refWidth{};
    int refHeight{};

    /// @brief The maximum allowed reprojection error to treat a point pair as an inlier
    /// Here we are using the RANSAC to remove outlier. That is if:
    /// \f[ \left|| dstPoints_i - convertPointHomogenous \left( H * srcPoints_i \right) \right|| > ransacReprojThreshold \f]
    /// then the point i is considered an outlier. If srcPoints and dstPoints are measured in pixels, it usually makes sense to set this parameter somewhere in the range of 1 to 10.
    double m_ransacReprojThreshold = 8.0;
};

}
}
}

#endif // SolARHomographyEstimationOpencv_H
