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

#include "SolARFundamentalMatrixDecompositionValidationOpencv.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include "ComponentFactory.h"

#include <map>

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARFundamentalMatrixDecompisitionValidationOpencv);

namespace SolAR {
using namespace datastructure;
    namespace MODULES {
        namespace OPENCV {
            SolARFundamentalMatrixDecompisitionValidationOpencv::SolARFundamentalMatrixDecompisitionValidationOpencv(){
                setUUID(SolARFundamentalMatrixDecompisitionValidationOpencv::UUID);
                addInterface<api::solver::pose::IFundamentalMatrixDecompositionValidation>(this,api::solver::pose::IFundamentalMatrixDecompositionValidation::UUID, "interface api::solver::pose::IFundamentalMatrixDecompositionValidation");
                LOG_DEBUG("SolARFundamentalMatrixValidationOpencv constructor")
            }

            bool SolARFundamentalMatrixDecompisitionValidationOpencv::isValid(const std::vector<SRef<Point2Df>>&pt2d_v1,
                                                                               const std::vector<SRef<Point2Df>>&pt2d_v2,
                                                                               const CamCalibration&K,
                                                                               const Pose&P1,
                                                                               const Pose&P2,
                                                                               const std::vector<SRef<Point3Df>>&pt3d){
                return true;
            }

        }
    }
}
