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

#ifndef SOLARIMAGEFILTERBLUROPENCV_H
#define SOLARIMAGEFILTERBLUROPENCV_H

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
 * @class SolARImageFilterBlurOpencv
 * @brief <B>Blurs an image using the normalized box filter.</B>
 * <TT>UUID: deb083aa-69fb-409a-af94-151d476de922</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ kernel_id,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 0 }}
 * @SolARComponentProperty{ kernel_width,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 0 }}
 * @SolARComponentProperty{ kernel_height,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 0 }}
 * @SolARComponentProperty{ direction,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ int, [0 (Homogeneous blurring)\, 1 (Gaussian blurring)\, 2 (Median blurring)\, 3 (Bilateral blurring)\, 4 (Bilateral blurring)], 0 }}
 * 
 * @SolARComponentPropertiesEnd
 */

class SOLAROPENCV_EXPORT_API SolARImageFilterBlurOpencv : public org::bcom::xpcf::ConfigurableBase,public api::image::IImageFilter {
public:

    SolARImageFilterBlurOpencv();
   ~SolARImageFilterBlurOpencv();

    FrameworkReturnCode filter(const SRef<datastructure::Image>input,SRef<datastructure::Image>& output) override;

    void unloadComponent () override final;

 private:
    int kernel_id;
    int kernel_width;
    int kernel_height;
    int direction;

};
}
}
}
#endif
