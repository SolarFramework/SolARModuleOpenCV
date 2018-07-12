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

#ifndef SolARHomographyMatrixMatrixDecomposerOpencv_H
#define SolARHomographyMatrixMatrixDecomposerOpencv_H
#include <vector>

#include "api/solver/pose/I2Dto3DTransformDecomposer.h"

#include "xpcf/component/ComponentBase.h"
#include "SolAROpencvAPI.h"
#include <vector>
#include "opencv2/core.hpp"

namespace SolAR {
    using namespace datastructure;
    namespace MODULES {
        namespace OPENCV {
            class SOLAROPENCV_EXPORT_API SolARHomographyMatrixDecomposerOpencv : public org::bcom::xpcf::ComponentBase,
             public api::solver::pose::I2DTO3DTransformDecomposer
                 {
                    public:
                        SolARHomographyMatrixDecomposerOpencv();
                        bool decompose(const Transform2Df&F,
                                       const CamCalibration&K,
                                       const CamDistortion& dist,
                                       std::vector<Transform3Df>& decomposedPoses) override;
                        void unloadComponent () override final;
                    private:
                };
        }
    }
 }
#endif // SOLARHOMOGRAPHYMATRIXDECOMPOSITIONOPENCV_H



