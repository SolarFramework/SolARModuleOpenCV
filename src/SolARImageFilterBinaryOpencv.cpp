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

#include "SolARImageFilterBinaryOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageFilterBinaryOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARImageFilterBinaryOpencv::SolARImageFilterBinaryOpencv():ConfigurableBase(xpcf::toUUID<SolARImageFilterBinaryOpencv>())
{
    // initialize max default value for the component
    m_max = 255;

    declareInterface<api::image::IImageFilter>(this);
    declareProperty("min", m_min);
    declareProperty("max", m_max);
}


SolARImageFilterBinaryOpencv::~SolARImageFilterBinaryOpencv(){

}


FrameworkReturnCode SolARImageFilterBinaryOpencv::filter(const SRef<Image>input, SRef<Image>& output){

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
    if (m_min>=0)
        cv::threshold(imgSource, imgFiltred, m_min, m_max, cv::THRESH_BINARY);
    else
        cv::threshold(imgSource, imgFiltred, 0, m_max, cv::THRESH_BINARY | cv::THRESH_OTSU);

    return FrameworkReturnCode::_SUCCESS;
}


}
}
}


