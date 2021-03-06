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

#ifndef SolARImageConvertorOpencv_H
#define SolARImageConvertorOpencv_H

#include "api/image/IImageConvertor.h"

// Definition of SolARImageConvertorOpencv Class //
// part of Solar namespace //

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include <string>

//#include "opencv/cv.h"
#include "opencv2/imgproc/imgproc.hpp"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARImageConvertorOpencv
 * @brief <B>Converts an image according to a given expected layout.</B>
 * <TT>UUID: fd7fb607-144f-418c-bcf2-f7cf71532c22</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ colorMap,
 *                          color map used\, list of GNU Octave/MATLAB equivalent colormaps available https://docs.opencv.org/4.1.1/d3/d50/group__imgproc__colormap.html,
 *                          @SolARComponentPropertyDescNum{ int, values of cv::ColormapTypes, cv::COLORMAP_JET (2) }}
 * @SolARComponentProperty{ equalizeLUT,
 *                             ,
 *                             @SolARComponentPropertyDescNum{ int, [0..1], 0 (false) }}
 * @SolARComponentPropertiesEnd
 */

class SOLAROPENCV_EXPORT_API SolARImageConvertorOpencv : public org::bcom::xpcf::ConfigurableBase,
        public api::image::IImageConvertor {
public:
    SolARImageConvertorOpencv();
    ~SolARImageConvertorOpencv();

    FrameworkReturnCode convert(const SRef<datastructure::Image> imgSrc, SRef<datastructure::Image>& imgDst) override;
    FrameworkReturnCode convert(const SRef<datastructure::Image> imgSrc, SRef<datastructure::Image>& imgDst, datastructure::Image::ImageLayout destLayout) override;
	FrameworkReturnCode convertLookUpTable(const SRef<datastructure::Image> imgSrc, SRef<datastructure::Image> & imgDst) override;

    void unloadComponent () override final;

private:
	//@brief color map used, list of GNU Octave/MATLAB equivalent colormaps available https://docs.opencv.org/4.1.1/d3/d50/group__imgproc__colormap.html
	int m_colorMap = cv::COLORMAP_JET; 
	int m_equalizeLUT = false;
};

}
}
}  // end of namespace Solar



#endif // SolARImageConvertorOpencv_H
