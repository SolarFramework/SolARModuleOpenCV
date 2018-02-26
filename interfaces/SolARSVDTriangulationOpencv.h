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


#ifndef SOLARSVDTRIANGULATIONOPENCV_H
#define SOLARSVDTRIANGULATIONOPENCV_H

#include "ComponentBase.h"
#include "SolAROpencvAPI.h"

#include "api/solver/map/ITriangulator.h"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARSVDTriangulationOpencv : public org::bcom::xpcf::ComponentBase,
        public api::solver::map::ITriangulator {
public:

    SolARSVDTriangulationOpencv();
   ~SolARSVDTriangulationOpencv();



    cv::Mat_<double> iterativeLinearTriangulation(cv::Point3d &u,
                                                  cv::Matx34d&P,
                                                  cv::Point3d&u1,
                                                  cv::Matx34d&P1);

    cv::Mat_<double> linearTriangulation(cv::Point3d &u,
                                                  cv::Matx34d&P,
                                                  cv::Point3d&u1,
                                                  cv::Matx34d&P1);


    bool triangulate(const std::vector<SRef<Point2Df>>& pt2d_1,
                     const std::vector<SRef<Point2Df>>& pt2d_2,
                     const SRef<Pose>&p1,
                     const SRef<Pose>&p2,
                     const CamCalibration&cam,
                     const CamDistortion&dist,
                     std::vector<SRef<Point3Df>>& pt3d);

    void unloadComponent () override final;

        XPCF_DECLARE_UUID(" 85274ecd-2914-4f12-96de-37c6040633a4");

 private:

};

}
}
}




#endif // SOLARMATCHESFILTEROPENCV_H
