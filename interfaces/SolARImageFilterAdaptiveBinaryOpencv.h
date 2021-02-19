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

#ifndef SOLARIMAGEFILTERADAPTIVEBINARYOPENCV_H
#define SOLARIMAGEFILTERADAPTIVEBINARYOPENCV_H

#include "api/image/IImageFilter.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARImageFilterAdaptiveBinaryOpencv
 * @brief <B>Filters a greyscale image to a binary image based on an adaptive threshold.</B>
 * <TT>UUID: 901e7a07-5013-4907-be41-0259fca3726c</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ max,
 *                          Non - zero value assigned to the pixels for which the condition is satisfied,
 *                          @SolARComponentPropertyDescNum{ int, [0..255], 255 }}
 * @SolARComponentProperty{ blockSize,
 *                          Size of a pixel neighborhood that is used to calculate a threshold value for the<br>
 *                            pixel : 3\, 5\, 7\, and so on.,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 11 }}
 * @SolARComponentProperty{ C,
 *                          Constant subtracted from the mean or weighted mean(see the details below).Normally\, it<br>
 *                            is positive but may be zero or negative as well.,
 *                          @SolARComponentPropertyDescNum{ int, [MIN INT..MAX INT], 2 }}
 * @SolARComponentPropertiesEnd
 * 
 */

class SOLAROPENCV_EXPORT_API SolARImageFilterAdaptiveBinaryOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::image::IImageFilter {
public:

    SolARImageFilterAdaptiveBinaryOpencv();
   ~SolARImageFilterAdaptiveBinaryOpencv();

    FrameworkReturnCode filter(const SRef<datastructure::Image>input, SRef<datastructure::Image>& output) override;

    void unloadComponent () override final;

 private:
    int m_max = 255;
    int m_blockSize = 11;
    int m_C = 2;
};

}
}
}

#endif
