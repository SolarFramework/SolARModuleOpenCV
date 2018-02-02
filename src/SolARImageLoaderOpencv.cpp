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
#include <iostream>
#include <utility>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdexcept>
#include <vector>

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImageLoaderOpencv);

using namespace org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARImageLoaderOpencv::SolARImageLoaderOpencv()
{
    setUUID(SolARImageLoaderOpencv::UUID);
    addInterface<api::image::IImageLoader>(this,api::image::IImageLoader::UUID, "interface api::image::IImageLoader");
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

FrameworkReturnCode SolARImageLoaderOpencv::loadImage(const std::string & filename, SRef<Image> & img)
{

    cv::Mat img_src = cv::imread(filename);
    if (img_src.data == NULL)
        return FrameworkReturnCode::_ERROR_LOAD_IMAGE;
    return SolAROpenCVHelper::convertToSolar(img_src,img);
}

}
}
}  // end of namespace SolAR
