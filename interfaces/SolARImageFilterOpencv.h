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

#ifndef SOLARIMAGEFILTEROPENCV_H
#define SOLARIMAGEFILTEROPENCV_H

#include "api/image/IImageFilter.h"

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

class SOLAROPENCV_EXPORT_API SolARImageFilterOpencv : public org::bcom::xpcf::ComponentBase,
        public api::image::IImageFilter {
public:

    SolARImageFilterOpencv();
   ~SolARImageFilterOpencv();

    FrameworkReturnCode threshold(SRef<Image>input,
                   SRef<Image>& output,
                   int threshold) override;

    FrameworkReturnCode binarize(SRef<Image>input,
                  SRef<Image>& output,
                  int min,
                  int max) override;

    FrameworkReturnCode adaptiveBinarize(SRef<Image>input,
                   SRef<Image>& output,
                   int max,
                   int blockSize,
                   int C) override;

    FrameworkReturnCode blur(SRef<Image>input,
              SRef<Image>& output,
              int kernerl_id,
              int kernel_width,
              int kernel_height,
              int direction) override;

    FrameworkReturnCode gradient(SRef<Image>input,
                  SRef<Image>& output,
                  int x_order,
                  int y_order) override;

    FrameworkReturnCode laplacian(SRef<Image>input,
                   SRef<Image>& output,
                   int method) override;

    FrameworkReturnCode erode(SRef<Image>input,
               SRef<Image>& output,
               int erosion_type,
               int erosion_size) override;


    FrameworkReturnCode dilate(SRef<Image>input,
                SRef<Image>& output,
                int dilatation_type,
                int dilatation_size) override;

    FrameworkReturnCode equalize(SRef<Image>input,
                   SRef<Image>& output,
                   int method) override;

    void unloadComponent () override final;

 private:

};

}
}
}

#endif
