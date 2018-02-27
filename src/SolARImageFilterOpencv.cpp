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

#include "SolARImageFilterOpencv.h"
#include "ComponentFactory.h"
#include "SolAROpenCVHelper.h"


namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageFilterOpencv);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARImageFilterOpencv::SolARImageFilterOpencv()
{
    setUUID(SolARImageFilterOpencv::UUID);
    addInterface<api::image::IImageFilter>(this,api::image::IImageFilter::UUID, "interface ImageFilterOpencv");
}


SolARImageFilterOpencv::~SolARImageFilterOpencv(){

}

FrameworkReturnCode SolARImageFilterOpencv::threshold(SRef<Image>input,
                   SRef<Image>& output,
                   int threshold){
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARImageFilterOpencv::binarize(SRef<Image>input,
              SRef<Image>& output,
              int min,
              int max){
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
    //cv::Mat imgSource(input->getHeight(),output->getWidth(),CV_8UC1, input->data());
    //cv::Mat imgFiltred(output->getHeight(),output->getWidth(),CV_8UC1,output->data());
    if (min>=0)
        cv::threshold(imgSource, imgFiltred, min, max, cv::THRESH_BINARY);
    else
        cv::threshold(imgSource, imgFiltred, 0, max, cv::THRESH_BINARY | cv::THRESH_OTSU);

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARImageFilterOpencv::adaptiveBinarize(SRef<Image>input,
               SRef<Image>& output,
               int max,
               int blockSize,
               int C){

    LOG_DEBUG("<Image adaptive binarization>:")
    output->setSize(input->getWidth(),input->getHeight());
    cv::Mat imgSource(input->getHeight(),output->getWidth(),CV_8UC1, input->data());
    cv::Mat imgFiltred(output->getHeight(),output->getWidth(),CV_8UC1,output->data());
    cv::adaptiveThreshold(imgSource, imgFiltred, max, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, blockSize, C);

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARImageFilterOpencv::blur(SRef<Image>input,
          SRef<Image>& output,
          int kernerl_id,
          int kernel_width,
          int kernel_height,
          int direction){

    LOG_DEBUG("<Image blurring>:")
    output->setSize(input->getWidth(),input->getHeight());
    cv::Mat imgSource(input->getHeight(),output->getWidth(),CV_8UC3, input->data());
    cv::Mat imgFiltred(output->getHeight(),output->getWidth(),CV_8UC3,output->data());
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

FrameworkReturnCode SolARImageFilterOpencv::gradient(SRef<Image>input,
              SRef<Image>& output,
              int x_order,
              int y_order){


    /*
     * UNDER CONSTRUCTION
     *
    std::cerr<<"<Image gradient>:"<<std::endl;
    output->setSize(input->getWidth(),input->getHeight());
    cv::Mat imgSource(input->getHeight(),output->getWidth(),CV_8UC1, input->data());
    cv::imshow(("imgsource opencv"), imgSource);
    cv::waitKey(0);
    cv::Mat imgFiltred(output->getHeight(),output->getWidth(),CV_8UC1,output->data());
    cv::Sobel(imgSource, imgFiltred,CV_16S,x_order, y_order,3,1,0,cv::BORDER_DEFAULT);
    std::cout<<"imgfiltred depth: "<<imgFiltred.depth()<<std::endl;
    std::cout<<"imgfiltred type: "<<imgFiltred.type()<<std::endl;
    */
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARImageFilterOpencv::laplacian(SRef<Image>input,
               SRef<Image>& output,
               int method){
    /*
     * UNDER CONSTRUCTION
     */
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARImageFilterOpencv::erode(SRef<Image>input,
               SRef<Image>& output,
               int erosion_elem,
               int erosion_size){
    LOG_ERROR("<Image erosion>:")
    output->setSize(input->getWidth(),input->getHeight());
    cv::Mat imgSource(input->getHeight(),output->getWidth(),CV_8UC3, input->data());
    cv::Mat imgFiltred(output->getHeight(),output->getWidth(),CV_8UC3,output->data());


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


FrameworkReturnCode SolARImageFilterOpencv::dilate(SRef<Image>input,
                                    SRef<Image>& output,
                                    int dilate_elem,
                                    int dilate_size ){

    LOG_DEBUG("<Image dilatation>:")
    output->setSize(input->getWidth(),input->getHeight());
    cv::Mat imgSource(input->getHeight(),output->getWidth(),CV_8UC3, input->data());
    cv::Mat imgFiltred(output->getHeight(),output->getWidth(),CV_8UC3,output->data());


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

FrameworkReturnCode SolARImageFilterOpencv::equalize(SRef<Image>input,
               SRef<Image>& output,
               int method){
    /*
     * UNDER CONSTRUCTION
       */
    return FrameworkReturnCode::_SUCCESS;
}

}
}
}


