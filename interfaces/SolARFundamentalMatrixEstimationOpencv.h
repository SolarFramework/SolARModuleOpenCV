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
#include "ComponentBase.h"
#include "SolAROpencvAPI.h"
#include <vector>
#include "opencv2/core.hpp"

namespace SolAR {
    using namespace datastructure;
    namespace MODULES {
        namespace OPENCV {
        /// @class SolARFundamentalMatrixEstimationOpencv
            class SOLAROPENCV_EXPORT_API SolARFundamentalMatrixEstimationOpencv : public org::bcom::xpcf::ComponentBase,
              public api::solver::pose::I2DTransformFinder
                {
                    public:
                       /// @brief SolARFundamentalMatrixEstimationOpencv constructor.
                        SolARFundamentalMatrixEstimationOpencv();
                       /// @brief SolARFundamentalMatrixEstimationOpencv destructor.
                        ~SolARFundamentalMatrixEstimationOpencv();
                        /// @brief Find fundamental matrix from 2 sets of 2d_points. Th estimation is based on the opencv findFundamental algorithm.
                        /// @param[in] Set of 2d_points seen in view_1.
                        /// @param[in] Set of 2d_points seen in view_2.
                        /// @param[out] Estimated Fundamental transform matrix.
                        api::solver::pose::Transform2DFinder::RetCode find(const std::vector< SRef<Point2Df> >& srcPoints,
                                                                       const std::vector< SRef<Point2Df> >& dstPoints,
                                                                       Transform2Df & fundamental) override;

                    void unloadComponent () override final;

                private:
                    bool isFValid(const Transform2Df & F);
                };

            }
       }
}

#endif // SolARHomographyEstimationOpencv_H
