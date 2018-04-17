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
#include "ComponentBase.h"
#include "SolAROpencvAPI.h"
#include <vector>
#include "opencv2/core.hpp"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARHomographyEstimationOpencv : public org::bcom::xpcf::ComponentBase,
    public api::solver::pose::I2DTransformFinder
{
public:
    SolARHomographyEstimationOpencv();

    api::solver::pose::Transform2DFinder::RetCode find(const std::vector< SRef<Point2Df> >& srcPoints,
                  const std::vector< SRef<Point2Df> >& dstPoints,
                  Transform2Df & homography) override;

    void unloadComponent () override final;


    XPCF_DECLARE_UUID("fb9dac20-2a44-44b2-aa42-2871eec31427");

private:
    bool isHValid(const Transform2Df & H);
    float computeSurface(std::vector<cv::Point2f> points);
    float computeDistance(cv::Point2f a, cv::Point2f b);

    std::vector<cv::Point2f> obj_corners;
    std::vector<cv::Point2f> scene_corners;
    int refWidth;
    int refHeight;
};

}
}
}

#endif // SolARHomographyEstimationOpencv_H
