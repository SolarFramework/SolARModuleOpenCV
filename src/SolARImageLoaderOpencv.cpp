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

#include "SolARImageLoaderOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include <opencv2/imgcodecs.hpp>

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageLoaderOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARImageLoaderOpencv::SolARImageLoaderOpencv():ConfigurableBase(xpcf::toUUID<SolARImageLoaderOpencv>())
{ 
    declareInterface<api::image::IImageLoader>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapString("filePath", m_filePath);
    LOG_DEBUG(" SolARImageLoaderOpencv constructor")
}


SolARImageLoaderOpencv::~SolARImageLoaderOpencv()
{
    LOG_DEBUG(" SolARImageLoaderOpencv destructor")
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

xpcf::XPCFErrorCode SolARImageLoaderOpencv::onConfigured()
{
    LOG_DEBUG(" SolARImageLoaderOpencv onConfigured");

    // Load the image when its path has been read
    if (reloadImage() == FrameworkReturnCode::_SUCCESS)
        return xpcf::_SUCCESS;
    else
        return xpcf::_FAIL;
}

FrameworkReturnCode SolARImageLoaderOpencv::reloadImage()
{
    // Load the image when its path has been read
    cv::Mat img_src = cv::imread(m_filePath);
    if (img_src.data == NULL)
        return FrameworkReturnCode::_ERROR_;
    return SolAROpenCVHelper::convertToSolar(img_src,m_img);
}

FrameworkReturnCode SolARImageLoaderOpencv::getImage(SRef<Image> & img)
{
   if (m_img == nullptr)
        return FrameworkReturnCode::_ERROR_;

   img = m_img;
   return FrameworkReturnCode::_SUCCESS;
}

}
}
}  // end of namespace SolAR
