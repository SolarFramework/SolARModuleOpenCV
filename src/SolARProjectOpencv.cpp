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

#include "SolARProjectOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include "opencv2/calib3d/calib3d.hpp"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARProjectOpencv);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARProjectOpencv::SolARProjectOpencv():ConfigurableBase(xpcf::toUUID<SolARProjectOpencv>())
{
    addInterface<api::geom::IProject>(this);

    m_camMatrix.create(3, 3, CV_32FC1);
    m_camDistorsion.create(5, 1, CV_32FC1);

    LOG_DEBUG(" SolARProjectOpencv constructor");
}

SolARProjectOpencv::~SolARProjectOpencv(){

}

FrameworkReturnCode SolARProjectOpencv::project(const std::vector<SRef<Point3Df>> & inputPoints, const Transform3Df& pose, std::vector<SRef<Point2Df>> & imagePoints)
{

    return FrameworkReturnCode::_SUCCESS;
}

void SolARProjectOpencv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {
    this->m_camDistorsion.at<float>(0, 0)  = distorsionParams(0);
    this->m_camDistorsion.at<float>(1, 0)  = distorsionParams(1);
    this->m_camDistorsion.at<float>(2, 0)  = distorsionParams(2);
    this->m_camDistorsion.at<float>(3, 0)  = distorsionParams(3);
    this->m_camDistorsion.at<float>(4, 0)  = distorsionParams(4);

    this->m_camMatrix.at<float>(0, 0) = intrinsicParams(0,0);
    this->m_camMatrix.at<float>(0, 1) = intrinsicParams(0,1);
    this->m_camMatrix.at<float>(0, 2) = intrinsicParams(0,2);
    this->m_camMatrix.at<float>(1, 0) = intrinsicParams(1,0);
    this->m_camMatrix.at<float>(1, 1) = intrinsicParams(1,1);
    this->m_camMatrix.at<float>(1, 2) = intrinsicParams(1,2);
    this->m_camMatrix.at<float>(2, 0) = intrinsicParams(2,0);
    this->m_camMatrix.at<float>(2, 1) = intrinsicParams(2,1);
    this->m_camMatrix.at<float>(2, 2) = intrinsicParams(2,2);
}

}
}
}
