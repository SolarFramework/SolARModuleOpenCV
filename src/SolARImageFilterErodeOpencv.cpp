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

#include "SolARImageFilterErodeOpencv.h"
#include "SolAROpenCVHelper.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageFilterErodeOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARImageFilterErodeOpencv::SolARImageFilterErodeOpencv():ConfigurableBase(xpcf::toUUID<SolARImageFilterErodeOpencv>())
{
    declareInterface<api::image::IImageFilter>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapInteger("erosion_elem", erosion_elem);
    params->wrapInteger("erosion_size", erosion_size);
}


SolARImageFilterErodeOpencv::~SolARImageFilterErodeOpencv(){

}


FrameworkReturnCode SolARImageFilterErodeOpencv::filter(const SRef<Image>input, SRef<Image>& output){

    if (output == nullptr)
        output = xpcf::utils::make_shared<Image> (input->getImageLayout(), input->getPixelOrder(), input->getDataType());

    output->setSize(input->getWidth(),input->getHeight());

    cv::Mat imgSource, imgFiltred;
    SolAROpenCVHelper::mapToOpenCV(input,imgSource);
    SolAROpenCVHelper::mapToOpenCV(output,imgFiltred);

    int erosion_type;
    if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
    else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
    else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }

    cv::Mat element = cv::getStructuringElement( erosion_type,
                                          cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                          cv::Point( erosion_size, erosion_size ) );


    cv::erode(imgSource,imgFiltred,element);

    return FrameworkReturnCode::_SUCCESS;
}


}
}
}


