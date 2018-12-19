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

#include "SolARImageConvertorOpencv.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/highgui/highgui.hpp"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageConvertorOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARImageConvertorOpencv::SolARImageConvertorOpencv():ComponentBase(xpcf::toUUID<SolARImageConvertorOpencv>())
{  
    addInterface<api::image::IImageConvertor>(this);
}


SolARImageConvertorOpencv::~SolARImageConvertorOpencv()
{

}

static std::map<std::pair<Image::ImageLayout,Image::ImageLayout>,int> convertMapInfos = {
    {{Image::ImageLayout::LAYOUT_RGB,Image::ImageLayout::LAYOUT_GREY},CV_RGB2GRAY},
    {{Image::ImageLayout::LAYOUT_BGR,Image::ImageLayout::LAYOUT_GREY},CV_BGR2GRAY}
};

inline int deduceOpenCVConversionMode(SRef<Image> imgSrc, SRef<Image> imgDst)
{
    // TODO : handle safe mode if missing map entry
    return convertMapInfos.at(std::make_pair<Image::ImageLayout,Image::ImageLayout>(imgSrc->getImageLayout(),imgDst->getImageLayout()));
}

inline int deduceOpenCVConversionMode(SRef<Image> imgSrc, Image::ImageLayout dstLayout)
{
    std::pair<Image::ImageLayout,Image::ImageLayout> key(imgSrc->getImageLayout(),dstLayout);
    // TODO : handle safe mode if missing map entry
    return convertMapInfos.at(key);
}

FrameworkReturnCode SolARImageConvertorOpencv::convert(SRef<Image> imgSrc, SRef<Image>& imgDst, Image::ImageLayout destLayout, const float scale)
{
    if (imgDst == nullptr)
        imgDst = xpcf::utils::make_shared<Image> (destLayout, imgSrc->getPixelOrder(), imgSrc->getDataType());

    imgDst->setSize(imgSrc->getWidth(),imgSrc->getHeight());

    cv::Mat imgSource, imgConverted;
    SolAROpenCVHelper::mapToOpenCV(imgSrc,imgSource);
    SolAROpenCVHelper::mapToOpenCV(imgDst,imgConverted);
    bool processed = false;
    if( imgSrc->getImageLayout() != destLayout )
    {
        cv::cvtColor(imgSource, imgConverted, deduceOpenCVConversionMode(imgSrc,destLayout), scale);
        processed = true;
    }
    if( scale != 1.f )
    {
        cv::Mat imgTmp(imgSource.rows,
                       imgSource.cols,
                       CV_32F);

        processed ? imgConverted.convertTo( imgTmp, CV_32F, scale ) :
                    imgSource.convertTo( imgTmp, CV_32F, scale );
        imgTmp.convertTo( imgConverted, imgConverted.type() );
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARImageConvertorOpencv::convert(SRef<Image> imgSrc, SRef<Image>& imgDst, const float scale)
{
   if (imgDst == nullptr)
   {
       LOG_ERROR("The imgDst has not been instantiated before calling convert method. Pleae, instantiate it or call the convert method that takes in argument the layout of the output image.")
       return FrameworkReturnCode::_ERROR_;
   }
   return convert(imgSrc,imgDst,imgDst->getImageLayout(),scale);
}

}
}
}  // end of namespace Solar
