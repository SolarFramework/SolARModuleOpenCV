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

#include "SolARImageFilterBlurOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageFilterBlurOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARImageFilterBlurOpencv::SolARImageFilterBlurOpencv():ConfigurableBase(xpcf::toUUID<SolARImageFilterBlurOpencv>())
{
    addInterface<api::image::IImageFilter>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapInteger("kernel_id", kernel_id);
    params->wrapInteger("kernel_width", kernel_width);
    params->wrapInteger("kernel_height", kernel_height);
    params->wrapInteger("direction", direction);
}


SolARImageFilterBlurOpencv::~SolARImageFilterBlurOpencv(){

}


FrameworkReturnCode SolARImageFilterBlurOpencv::filter(const SRef<Image>input, SRef<Image>& output){

    if (output == nullptr)
        output = xpcf::utils::make_shared<Image> (input->getImageLayout(), input->getPixelOrder(), input->getDataType());

    output->setSize(input->getWidth(),input->getHeight());
    cv::Mat imgSource, imgFiltred;
    SolAROpenCVHelper::mapToOpenCV(input,imgSource);
    SolAROpenCVHelper::mapToOpenCV(output,imgFiltred);

    switch (direction) {
    case 0:{
        LOG_DEBUG("    <Homogeneous blurring>:")
        cv::blur(imgSource, imgFiltred, cv::Size(kernel_width, kernel_height), cv::Point(-1,-1));
        break;
   }
    case 1:{
        LOG_DEBUG("    <Gaussian blurring>:")
        cv::GaussianBlur(imgSource, imgFiltred, cv::Size(kernel_width, kernel_height),0,0);
        break;
   }
    case 2:{
        LOG_DEBUG("    <Median blurring>:")
        cv::medianBlur(imgSource, imgFiltred, kernel_width);
        break;
   }
    case 3:{
        LOG_DEBUG("    <Bilateral blurring>:")
        cv::bilateralFilter(imgSource, imgFiltred,kernel_width, kernel_width*2, kernel_height/2);
        break;
   }
    default:
        break;
    }

    return FrameworkReturnCode::_SUCCESS;
}


}
}
}


