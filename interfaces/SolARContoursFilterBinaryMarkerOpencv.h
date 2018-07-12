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

#ifndef SOLARCONTOURSFILTERBINARYMARKEROPENCV_H
#define SOLARCONTOURSFILTERBINARYMARKEROPENCV_H

#include "api/features/IContoursFilter.h"

#include "xpcf/component/ComponentBase.h"
#include "SolAROpencvAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARContoursFilterBinaryMarkerOpencv : public org::bcom::xpcf::ComponentBase,
        public api::features::IContoursFilter {
public:
    SolARContoursFilterBinaryMarkerOpencv();
    ~SolARContoursFilterBinaryMarkerOpencv() = default;

    void setParameters (float m_minContourLength)  override;

    FrameworkReturnCode filter(const std::vector<SRef<Contour2Df>> & input_contours, std::vector<SRef<Contour2Df>> & filtered_contours) override;

    void unloadComponent () override final; 

private:
    float m_minContourLength;
};

}
}
}  // end of namespace Solar

#endif // SOLARCONTOURSFILTERBINARYMARKEROPENCV_H
