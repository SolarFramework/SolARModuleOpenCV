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

#ifndef SOLARIMAGEFILTERERODEOPENCV_H
#define SOLARIMAGEFILTERERODEOPENCV_H

#include "api/image/IImageFilter.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARImageFilterErodeOpencv
 * @brief <B>Erodes the white regions of a binary image.</B>
 * <TT>UUID: 58b09819-64bc-4a80-b6a2-9fe7b179f3fc</TT>
 *
 */

class SOLAROPENCV_EXPORT_API SolARImageFilterErodeOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::image::IImageFilter {
public:

    SolARImageFilterErodeOpencv();
   ~SolARImageFilterErodeOpencv();

    FrameworkReturnCode filter(const SRef<Image>input,
                  SRef<Image>& output) override;

    void unloadComponent () override final;

 private:
    int erosion_elem, erosion_size;
};

}
}
}

#endif
