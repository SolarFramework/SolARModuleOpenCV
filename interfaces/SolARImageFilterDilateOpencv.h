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

#ifndef SOLARIMAGEFILTERDILATEOPENCV_H
#define SOLARIMAGEFILTERDILATEOPENCV_H

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
 * @class SolARImageFilterDilateOpencv
 * @brief <B>Dilates the white regions of a binary image.</B>
 * <TT>UUID: 7ac9d1b8-afda-4c99-b8df-92e71015a3be</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ dilate_elem,
 *                          See cv::MorphShapes,
 *                          @SolARComponentPropertyDescNum{ int, [0 (cv::MORPH_RECT)\, 1 (cv::MORPH_RECT)\, 2 (cv::MORPH_RECT)], 0 }}
 * @SolARComponentProperty{ dilate_size,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ int, [0.. MAX INT], 0 }}
 * @SolARComponentPropertiesEnd
 * 
 */

class SOLAROPENCV_EXPORT_API SolARImageFilterDilateOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::image::IImageFilter {
public:

    SolARImageFilterDilateOpencv();
   ~SolARImageFilterDilateOpencv();

    FrameworkReturnCode filter(const SRef<datastructure::Image>input,
                  SRef<datastructure::Image>& output) override;

    void unloadComponent () override final;

 private:
    int dilate_elem, dilate_size;
};

}
}
}

#endif
