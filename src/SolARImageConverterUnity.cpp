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

#include "SolARImageConverterUnity.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/highgui/highgui.hpp"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageConverterUnity)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARImageConverterUnity::SolARImageConverterUnity():ComponentBase(xpcf::toUUID<SolARImageConverterUnity>())
{  
    declareInterface<api::image::IImageConverter>(this);
}


SolARImageConverterUnity::~SolARImageConverterUnity()
{

}

static std::map<std::pair<Image::ImageLayout,Image::ImageLayout>,int> convertMapInfos = {{{Image::ImageLayout::LAYOUT_RGB,Image::ImageLayout::LAYOUT_GREY},CV_RGB2GRAY},
                                                                                       {{Image::ImageLayout::LAYOUT_BGR,Image::ImageLayout::LAYOUT_GREY},CV_BGR2GRAY}};

FrameworkReturnCode SolARImageConverterUnity::convert(SRef<Image> imgSrc, SRef<Image>& imgDst, Image::ImageLayout destLayout)
{
    cv::Mat opencvImage = SolAROpenCVHelper::mapToOpenCV(imgSrc);
    cv::flip(opencvImage, opencvImage,0);
    SolAROpenCVHelper::convertToSolar(opencvImage, imgDst);

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARImageConverterUnity::convert(SRef<Image> imgSrc, SRef<Image>& imgDst)
{
   return FrameworkReturnCode::_SUCCESS;
}

}
}
}  // end of namespace Solar
