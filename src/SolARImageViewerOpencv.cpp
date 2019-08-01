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
#include <opencv2/highgui.hpp>
#include "core/Log.h"


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageViewerOpencv)

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

static std::map<int,std::pair<Image::ImageLayout,Image::DataType>> cv2solarTypeConvertMap = {{CV_8UC3,{Image::ImageLayout::LAYOUT_BGR,Image::DataType::TYPE_8U}},
                                                                                             {CV_8UC1,{Image::ImageLayout::LAYOUT_GREY,Image::DataType::TYPE_8U}}};



static std::map<std::tuple<uint32_t,std::size_t,uint32_t>,int> solar2cvTypeConvertMap {{std::make_tuple(8,1,3),CV_8UC3},{std::make_tuple(8,1,1),CV_8UC1}};

inline int deduceOpenCVType(const SRef<Image>& img)
{
    // TODO : handle safe mode if missing map entry
    // is it ok when destLayout != img->ImageLayout ?
    return solar2cvTypeConvertMap.at(std::forward_as_tuple(img->getNbBitsPerComponent(),1,img->getNbChannels()));
}

SolARImageViewerOpencv::SolARImageViewerOpencv():ConfigurableBase(xpcf::toUUID<SolARImageViewerOpencv>())
{
    declareInterface<api::display::IImageViewer>(this);
    declareProperty("title", m_title);
    declareProperty("width", m_width);
    declareProperty("height", m_height);
    declareProperty("exitKey", m_exitKey);
    declareProperty("duration", m_duration);
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


FrameworkReturnCode SolARImageViewerOpencv::display(SRef<Image> img)
{
    char key=' ';
    cv::Mat imgSource(img->getHeight(),img->getWidth(),deduceOpenCVType(img), img->data());
    cv::namedWindow( m_title,0); // Create a window for display.
    if (m_isFirstDisplay)
    {
        if(m_width>0 && m_height>0)
            cv::resizeWindow(m_title, m_width,m_height);
        else
            cv::resizeWindow(m_title, img->getWidth(), img->getHeight());
        m_isFirstDisplay = false;
    }

    cv::imshow(m_title, imgSource);
    if (m_duration >0)
        key = cv::waitKey(m_duration);  // wait for a keystroke to display window
    else if (m_exitKey >= 0)
        key = cv::waitKey(10);  // wait for a keystroke to display window
    else
        key = cv::waitKey(1);  // wait for a keystroke to display window

    if(key == (char)(m_exitKey))
        return FrameworkReturnCode::_STOP;

    return FrameworkReturnCode::_SUCCESS;
}

}
}
}  // end of namespace SolAR
