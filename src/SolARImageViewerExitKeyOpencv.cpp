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

#include "SolARImageViewerExitKeyOpencv.h"
#include <iostream>
#include <utility>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdexcept>
#include <vector>

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageViewerExitKeyOpencv)

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

SolARImageViewerExitKeyOpencv::SolARImageViewerExitKeyOpencv():ConfigurableBase(xpcf::toUUID<SolARImageViewerExitKeyOpencv>())
{
    addInterface<api::display::IImageViewer>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapString("title", m_title);
    params->wrapInteger("width", m_width);
    params->wrapInteger("height", m_height);
    params->wrapInteger("exitKey", m_exitKey);

    LOG_DEBUG(" SolARImageViewerExitKeyOpencv constructor")
}


SolARImageViewerExitKeyOpencv::~SolARImageViewerExitKeyOpencv()
{
    LOG_DEBUG(" SolARImageViewerExitKeyOpencv destructor")
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

FrameworkReturnCode SolARImageViewerExitKeyOpencv::display(SRef<Image> img)
{
    char key=' ';
    cv::Mat imgSource(img->getHeight(),img->getWidth(),deduceOpenCVType(img), img->data());

    cv::namedWindow( m_title,0); // Create a window for display.
    if(m_width>0 && m_height>0)
        cv::resizeWindow(m_title, m_width,m_height);

    cv::imshow(m_title, imgSource);
    key=cv::waitKey(10); // wait for a keystroke to display window
    if(key == (char)(m_exitKey))
        return FrameworkReturnCode::_STOP;
    return FrameworkReturnCode::_SUCCESS;
}

}
}
}  // end of namespace SolAR
