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

#include "SolARImageFilterDilateOpencv.h"
#include "SolAROpenCVHelper.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageFilterDilateOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARImageFilterDilateOpencv::SolARImageFilterDilateOpencv():ConfigurableBase(xpcf::toUUID<SolARImageFilterDilateOpencv>())
{
    addInterface<api::image::IImageFilter>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapInteger("dilate_elem", dilate_elem);
    params->wrapInteger("dilate_size", dilate_size);
}


SolARImageFilterDilateOpencv::~SolARImageFilterDilateOpencv(){

}


FrameworkReturnCode SolARImageFilterDilateOpencv::filter(const SRef<Image>input,
              SRef<Image>& output){

    if (output == nullptr)
        output = xpcf::utils::make_shared<Image> (input->getImageLayout(), input->getPixelOrder(), input->getDataType());

    output->setSize(input->getWidth(),input->getHeight());

    cv::Mat imgSource, imgFiltred;
    SolAROpenCVHelper::mapToOpenCV(input,imgSource);
    SolAROpenCVHelper::mapToOpenCV(output,imgFiltred);

    int dilate_type;
    if(dilate_elem == 0 ){ dilate_type = cv::MORPH_RECT; }
    else if( dilate_elem == 1 ){ dilate_type = cv::MORPH_CROSS; }
    else if( dilate_elem == 2) { dilate_type = cv::MORPH_ELLIPSE; }

    cv::Mat element = cv::getStructuringElement( dilate_type,
                                          cv::Size( 2*dilate_size + 1, 2*dilate_size+1 ),
                                          cv::Point( dilate_size, dilate_size ) );


    cv::dilate(imgSource,imgFiltred,element);

    return FrameworkReturnCode::_SUCCESS;
}


}
}
}


