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

#include "SolARStereo2DPointsRectificationOpencv.h"
#include "core/Log.h"
#include "SolAROpenCVHelper.h"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARStereo2DPointsRectificationOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARStereo2DPointsRectificationOpencv::SolARStereo2DPointsRectificationOpencv() :base::geom::A2DPointsRectification(xpcf::toMap<SolARStereo2DPointsRectificationOpencv>())
{
    LOG_DEBUG("SolARStereo2DPointsRectificationOpencv constructor");
}

SolARStereo2DPointsRectificationOpencv::~SolARStereo2DPointsRectificationOpencv()
{
    LOG_DEBUG("SolARStereo2DPointsRectificationOpencv destructor");
}

FrameworkReturnCode SolARStereo2DPointsRectificationOpencv::rectify(const std::vector<SolAR::datastructure::Point2Df>& points2D,
															const SolAR::datastructure::CameraParameters& camParams,
															const SolAR::datastructure::RectificationParameters& rectParams, 
															std::vector<SolAR::datastructure::Point2Df>& rectifiedPoints2D)
{
    for (const auto& pt2D : points2D) {
        float u = pt2D.getX();
        float v = pt2D.getY();
        float x = (u - camParams.intrinsic(0, 2)) / camParams.intrinsic(0, 0);
        float y = (v - camParams.intrinsic(1, 2)) / camParams.intrinsic(1, 1);
        Maths::Vector3f pt3D(x, y, 1);
        Maths::Vector3f pt3DRect = rectParams.rotation * pt3D;
        float xRect = pt3DRect(0) / pt3DRect(2);
        float yRect = pt3DRect(1) / pt3DRect(2);
        float uRect = xRect * rectParams.projection(0, 0) + rectParams.projection(0, 2);
        float vRect = yRect * rectParams.projection(1, 1) + rectParams.projection(1, 2);
        rectifiedPoints2D.push_back(Point2Df(uRect, vRect));
	}
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}
