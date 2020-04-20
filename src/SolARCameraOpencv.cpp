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

#include "SolARCameraOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARCameraOpencv)

namespace SolAR {
namespace MODULES {
namespace OPENCV {

    SolARCameraOpencv::SolARCameraOpencv():SolARBaseCameraOpencv (xpcf::toUUID<SolARCameraOpencv>())
    {
        declareInterface<api::input::devices::ICamera>(this);
        declareProperty<uint32_t>("deviceID", m_deviceID);
    }

    SolARCameraOpencv::~SolARCameraOpencv()
    {
    }


    FrameworkReturnCode SolARCameraOpencv::getNextImage(SRef<Image> & img)
    {
        if (!m_capture.isOpened())
            return FrameworkReturnCode::_ERROR_ACCESS_IMAGE;
        cv::Mat cvFrame;
        m_capture >> cvFrame;
        if(!cvFrame.data)
            return FrameworkReturnCode::_ERROR_LOAD_IMAGE;

        return SolAROpenCVHelper::convertToSolar(cvFrame,img);
    }

    FrameworkReturnCode SolARCameraOpencv::start(){
        if(m_capture.isOpened())
        {
            m_capture.release();
        }
        m_capture = cv::VideoCapture( m_deviceID);
        if (m_capture.isOpened())
        {
            LOG_INFO("Camera with id {} has started", m_deviceID);
            LOG_INFO("Camera using {}  *  {} resolution", m_parameters.resolution.width ,m_parameters.resolution.height)
            if (m_is_resolution_set)
            {
                m_capture.set(cv::CAP_PROP_FRAME_WIDTH, m_parameters.resolution.width );
                m_capture.set(cv::CAP_PROP_FRAME_HEIGHT, m_parameters.resolution.height );
            }
            else {
                // set default resolution : get camera resolution ? or force camera resolution from default resolution values ?
            }
            return FrameworkReturnCode::_SUCCESS;
        }
        else
        {
            LOG_ERROR("Cannot open camera with id {}", m_deviceID);
            return FrameworkReturnCode::_ERROR_;
        }
    }

}
}
}

