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

#include "api/solver/pose/IFundamentalMatrixEstimation.h"
#include "ComponentBase.h"
#include "SolAROpencvAPI.h"
#include <vector>
#include "opencv2/core.hpp"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARFundamentalMatrixEstimationOpencv : public org::bcom::xpcf::ComponentBase,
    public api::solver::pose::IFundamentalMatrixEstimation
{
public:
    SolARFundamentalMatrixEstimationOpencv();

    api::solver::pose::FundamentalMatrixEstimation::RetCode findFundamental(const std::vector< SRef<Point2Df> >& srcPoints,
                  const std::vector< SRef<Point2Df> >& dstPoints,
                  Transform2Df & fundamental) override;

    void unloadComponent () override final;


    XPCF_DECLARE_UUID("79b29b50-cf4d-441e-b5de-1de829b91c41");

private:
    bool isFValid(const Transform2Df & F);

};

}
}
}

#endif // SolARHomographyEstimationOpencv_H
