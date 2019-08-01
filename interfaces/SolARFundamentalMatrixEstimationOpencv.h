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

#ifndef SolARFundamentalMatrixEstimationOpencv_H
#define SolARFundamentalMatrixEstimationOpencv_H
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
 * @class SolARFundamentalMatrixEstimationOpencv
 * @brief <B>Estimates the fundamental matrix from two set of keypoints that match together.</B>
 * <TT>UUID: 79b29b50-cf4d-441e-b5de-1de829b91c41</TT>
 *
 */

/// @class SolARFundamentalMatrixEstimationOpencv
class SOLAROPENCV_EXPORT_API SolARFundamentalMatrixEstimationOpencv : public org::bcom::xpcf::ConfigurableBase,
  public api::solver::pose::I2DTransformFinder {

public:
   /// @brief SolARFundamentalMatrixEstimationOpencv constructor.
    SolARFundamentalMatrixEstimationOpencv();

    /// @brief SolARFundamentalMatrixEstimationOpencv destructor.
    ~SolARFundamentalMatrixEstimationOpencv() override;

    /// @brief Find fundamental matrix from 2 sets of 2d_points. Th estimation is based on the opencv findFundamental algorithm.
    /// @param[in] srcPoints: set of source 2d points.
    /// @param[in] targetPoints set of target 2d points.
    /// @param[out] Estimated Fundamental transform matrix.
    /// @return Transform2DFinder::RetCode::TRANSFORM2D_ESTIMATION_OK if succeed.
    api::solver::pose::Transform2DFinder::RetCode find(const std::vector<Point2Df> & srcPoints,
                                                   const std::vector<Point2Df> & dstPoints,
                                                   Transform2Df & fundamental) override;

    void unloadComponent () final;

private:
    bool isFValid(const Transform2Df & F);

    /// @brief The desirable level of confidence (propability) that the estimated matrix is correct.
    float m_confidenceLevel = 0.99;

    ///  @brief threshold to define which point are ouliers
    ///  Here we are using a RANSAC method to remove outlier.
    ///  This attribute is the ratio between the maximum distance in pixels between source points and the maximum distance in pixels to the epipolar line for which point is considered as a outlier.
    ///  The higher is this ratio, the more you will keep inliers to estimate your 2D transform, but the less this estimation will be correct.
    ///  By default, this value is set to the one proposed by [Snavely07 4.1]
    float m_outlierDistanceRatio = 0.006;
};

}
}
} // end of namespace Solar

#endif // SolARHomographyEstimationOpencv_H
