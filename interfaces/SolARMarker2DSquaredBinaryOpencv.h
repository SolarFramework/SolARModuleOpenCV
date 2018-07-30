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

#ifndef SOLARMARKER2DSQUAREDBINARYOPENCV_H
#define SOLARMARKER2DSQUAREDBINARYOPENCV_H

#include "api/input/files/IMarker2DSquaredBinary.h"

#include "xpcf/component/ComponentBase.h"
#include "SolAROpencvAPI.h"

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARMarker2DSquaredBinaryOpencv : public org::bcom::xpcf::ComponentBase,
        public api::input::files::IMarker2DSquaredBinary{
public:

   SolARMarker2DSquaredBinaryOpencv();
   ~SolARMarker2DSquaredBinaryOpencv();

    FrameworkReturnCode loadMarker(const std::string & filename) override;

    void unloadComponent () override final;

};

}
}
}  // end of namespace Solar
#endif // SOLARMARKER2DSQUAREDBINARYOPENCV_H



