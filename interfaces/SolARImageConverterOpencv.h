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

#ifndef SolARImageConverterOpencv_H
#define SolARImageConverterOpencv_H

#include "api/image/IImageConverter.h"

// Definition of SolARImageConverterOpencv Class //
// part of Solar namespace //

#include "xpcf/component/ComponentBase.h"
#include "SolAROpencvAPI.h"
#include <string>

#include "opencv/cv.h"
#include "opencv2/imgproc/imgproc.hpp"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARImageConverterOpencv
 * @brief <B>Converts an image according to a given expected layout.</B>
 * <TT>UUID: fd7fb607-144f-418c-bcf2-f7cf71532c22</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARImageConverterOpencv : public org::bcom::xpcf::ComponentBase,
        public api::image::IImageConverter {
public:
    SolARImageConverterOpencv();
    ~SolARImageConverterOpencv();

    FrameworkReturnCode convert(SRef<Image> imgSrc, SRef<Image>& imgDst) override;
    FrameworkReturnCode convert(SRef<Image> imgSrc, SRef<Image>& imgDst, Image::ImageLayout destLayout) override;

    void unloadComponent () override final;

private:
};

}
}
}  // end of namespace Solar



#endif // SolARImageConverterOpencv_H
