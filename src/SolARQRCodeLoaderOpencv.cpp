/**
 * @copyright Copyright (c) 2020 B-com http://www.b-com.com/
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

#include "SolARQRCodeLoaderOpencv.h"
#include "core/Log.h"
#include "opencv2/opencv.hpp"
#include "datastructure/QRCode.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARQRCodeLoaderOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

    SolARQRCodeLoaderOpencv::SolARQRCodeLoaderOpencv():ConfigurableBase(xpcf::toUUID<SolARQRCodeLoaderOpencv>())
    {
        LOG_DEBUG("SolARQRCodeLoaderOpencv constructor")
        declareInterface<api::input::files::ITrackableLoader>(this);
        declareProperty("filePath", m_filePath);
    }

    SolARQRCodeLoaderOpencv::~SolARQRCodeLoaderOpencv()
    {
        LOG_DEBUG(" SolARQRCodeLoaderOpencv destructor")
    }

    FrameworkReturnCode SolARQRCodeLoaderOpencv::loadTrackable(SRef<Trackable>& trackable)
    {
        std::string code;
        Sizef markerSize;
        LOG_DEBUG("SolARQRCodeLoaderOpencv::loadTrackable");

        if (m_filePath.empty())
        {
            LOG_ERROR("QR code file path has not been defined");
            return FrameworkReturnCode::_ERROR_;
        }

        LOG_DEBUG("Load QR code from file: {}", m_filePath);

        cv::FileStorage fs(m_filePath, cv::FileStorage::READ);

        if (!fs.isOpened())
        {
            LOG_ERROR("Binary Fiducial Marker file {} cannot be loaded", m_filePath)
            return FrameworkReturnCode::_ERROR_;
        }
        fs["MarkerWidth"] >> markerSize.width;
        fs["MarkerHeight"] >> markerSize.height;
        fs["Code"] >> code;
        fs.release();

        // Create the new QRCode object (kind of Trackable object)
        trackable = xpcf::utils::make_shared<QRCode>(m_filePath, markerSize, code);

        return FrameworkReturnCode::_SUCCESS;
    }

}
}
}
