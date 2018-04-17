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

#ifndef SOLARPOSEESTIMATIONOPENCV_H
#define SOLARPOSEESTIMATIONOPENCV_H
#include <vector>
#include "opencv2/core.hpp"

#include "api/solver/pose/I3DTransformFinder.h"

#include "SolAROpencvAPI.h"
#include "ComponentBase.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARPoseEstimationOpencv : public org::bcom::xpcf::ComponentBase,
    public api::solver::pose::I3DTransformFinder
{
public:
    SolARPoseEstimationOpencv();

FrameworkReturnCode estimate(const std::vector<SRef<Point2Df>> & imagePoints,
                             const std::vector<SRef<Point3Df>> & worldPoints,
                             Pose & pose) override;

    void setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams)  override;
    void unloadComponent () override final;


    XPCF_DECLARE_UUID("0753ade1-7932-4e29-a71c-66155e309a53");

private:
    cv::Mat m_camMatrix;
    cv::Mat m_camDistorsion;

};

}
}
}

#endif // SOLARPOSEESTIMATIONOPENCV_H
