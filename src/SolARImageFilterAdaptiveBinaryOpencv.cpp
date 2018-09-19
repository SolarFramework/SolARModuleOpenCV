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

#include "SolARImageFilterAdaptiveBinaryOpencv.h"
#include "SolAROpenCVHelper.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageFilterAdaptiveBinaryOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARImageFilterAdaptiveBinaryOpencv::SolARImageFilterAdaptiveBinaryOpencv():ConfigurableBase(xpcf::toUUID<SolARImageFilterAdaptiveBinaryOpencv>())
{
    addInterface<api::image::IImageFilter>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapInteger("max", max);
    params->wrapInteger("blockSize", blockSize);
    params->wrapInteger("C", C);

}


SolARImageFilterAdaptiveBinaryOpencv::~SolARImageFilterAdaptiveBinaryOpencv(){

}


FrameworkReturnCode SolARImageFilterAdaptiveBinaryOpencv::filter(const SRef<Image>input, SRef<Image>& output){
    if (input->getImageLayout() != Image::ImageLayout::LAYOUT_GREY)
    {
        LOG_ERROR ("binarize method take as input only Grey images");
        return FrameworkReturnCode::_ERROR_;
    }

    if (output == nullptr)
        output = xpcf::utils::make_shared<Image> (Image::ImageLayout::LAYOUT_GREY, input->getPixelOrder(), input->getDataType());

    output->setSize(input->getWidth(),input->getHeight());

    cv::Mat imgSource, imgFiltred;
    SolAROpenCVHelper::mapToOpenCV(input,imgSource);
    SolAROpenCVHelper::mapToOpenCV(output,imgFiltred);

    cv::adaptiveThreshold(imgSource, imgFiltred, max, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, blockSize, C);

    return FrameworkReturnCode::_SUCCESS;
}


}
}
}


