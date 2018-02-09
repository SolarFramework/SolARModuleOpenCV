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

#include "SolARImageViewerOpencv.h"
#include <iostream>
#include <utility>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdexcept>
#include <vector>

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageViewerOpencv);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

static std::map<int,std::pair<Image::ImageLayout,Image::DataType>> cv2solarTypeConvertMap = {{CV_8UC3,{Image::ImageLayout::LAYOUT_BGR,Image::DataType::TYPE_8U}},
                                                                                             {CV_8UC1,{Image::ImageLayout::LAYOUT_GREY,Image::DataType::TYPE_8U}}};



static std::map<std::tuple<uint32_t,std::size_t,uint32_t>,int> solar2cvTypeConvertMap {{std::make_tuple(8,1,3),CV_8UC3},{std::make_tuple(8,1,1),CV_8UC1}};

inline int deduceOpenCVType(SRef<Image> img)
{
    // TODO : handle safe mode if missing map entry
    // is it ok when destLayout != img->ImageLayout ?
    return solar2cvTypeConvertMap.at(std::forward_as_tuple(img->getNbBitsPerComponent(),1,img->getNbChannels()));
}

SolARImageViewerOpencv::SolARImageViewerOpencv()
{
    setUUID(SolARImageViewerOpencv::UUID);
    addInterface<api::display::IImageViewer>(this,api::display::IImageViewer::UUID, "interface api::display::IImageViewer");
    LOG_DEBUG(" SolARImageViewerOpencv constructor")
}


SolARImageViewerOpencv::~SolARImageViewerOpencv()
{
    LOG_DEBUG(" SolARImageViewerOpencv destructor")
}

static std::map<long,FrameworkReturnCode> OpenCVImageCodeMap = {
    {0,FrameworkReturnCode::_SUCCESS},
    {-1,FrameworkReturnCode::_ERROR_},
    {-10,FrameworkReturnCode::_ERROR_LOAD_IMAGE},
    {-11,FrameworkReturnCode::_ERROR_LOAD_IMAGE},
    {-21,FrameworkReturnCode::_ERROR_ACCESS_IMAGE},
};

static FrameworkReturnCode safeErrorCodeConvert(int errCode)
{
    if (OpenCVImageCodeMap.find(errCode) == OpenCVImageCodeMap.end()) {
        return FrameworkReturnCode::_ERROR_;
    }
    return OpenCVImageCodeMap[errCode];
}

FrameworkReturnCode SolARImageViewerOpencv::display(const char * title, SRef<Image> img)
{
    cv::Mat imgSource(img->getHeight(),img->getWidth(),deduceOpenCVType(img), img->data());
    cv::namedWindow( title,CV_WINDOW_AUTOSIZE); // Create a window for display.

    cv::imshow(title, imgSource);
    cv::waitKey(1); // wait for a keystroke to display window
    return FrameworkReturnCode::_SUCCESS;
}


FrameworkReturnCode SolARImageViewerOpencv::display(const char * title, SRef<Image> img,int w_window, int h_window)
{
    cv::Mat imgSource(img->getHeight(),img->getWidth(),deduceOpenCVType(img), img->data());
    cv::namedWindow( title,0); // Create a window for display.
    cv::resizeWindow(title, w_window,h_window);

    cv::imshow(title, imgSource);
    cv::waitKey(1); // wait for a keystroke to display window
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARImageViewerOpencv::display(const char * title, SRef<Image> img, int w_window, int h_window, const char* exitKey)
{
    char key=' ';
    cv::Mat imgSource(img->getHeight(),img->getWidth(),deduceOpenCVType(img), img->data());

    cv::namedWindow( title,0); // Create a window for display.
    cv::resizeWindow(title, w_window,h_window);

    cv::imshow(title, imgSource);
    key=cv::waitKey(10); // wait for a keystroke to display window
    if(key == *exitKey)
        return FrameworkReturnCode::_STOP;
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARImageViewerOpencv::display(const char * title, SRef<Image> img,int w_window, int h_window, uint32_t duration)
{
    cv::Mat imgSource(img->getHeight(),img->getWidth(),deduceOpenCVType(img), img->data());

    cv::namedWindow( title,0); // Create a window for display.
    cv::resizeWindow(title, w_window,h_window);

    cv::imshow(title, imgSource);
    cv::waitKey(duration); // wait for a keystroke to display window
    return FrameworkReturnCode::_SUCCESS;
}

}
}
}  // end of namespace SolAR
