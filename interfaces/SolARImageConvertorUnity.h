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

#ifndef SolARImageConvertorUnity_H
#define SolARImageConvertorUnity_H

#include "api/image/IImageConvertor.h"

// Definition of SolARImageConvertorUnity Class //
// part of Solar namespace //

#include "xpcf/component/ComponentBase.h"
#include "SolAROpencvAPI.h"
#include <string>

#include "opencv2/imgproc/imgproc.hpp"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARImageConvertorUnity
 * @brief <B>Converts an image to be compliant with Unity image format and layout.</B>
 * <TT>UUID: 65282fb3-6651-4e73-b532-5a64ade0ead0</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARImageConvertorUnity : public org::bcom::xpcf::ComponentBase,
        public api::image::IImageConvertor {
public:
    SolARImageConvertorUnity();
    ~SolARImageConvertorUnity();

    FrameworkReturnCode convert(SRef<Image> imgSrc, SRef<Image>& imgDst) override;
    FrameworkReturnCode convert(SRef<Image> imgSrc, SRef<Image>& imgDst, Image::ImageLayout destLayout) override;

    void unloadComponent () override final;

private:
};

}
}
}  // end of namespace Solar



#endif // SolARImageConvertorOpencv_H
