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

#include "SolARVideoAsCameraOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARVideoAsCameraOpencv)

namespace SolAR {
    namespace MODULES {
    namespace OPENCV {

    SolARVideoAsCameraOpencv::SolARVideoAsCameraOpencv():SolARBaseCameraOpencv(xpcf::toUUID<SolARVideoAsCameraOpencv>())
    {
        declareInterface<api::input::devices::ICamera>(this);
        declareProperty("videoPath", m_videoPath);
        m_is_resolution_set = false;
    }



    FrameworkReturnCode SolARVideoAsCameraOpencv::getNextImage(SRef<Image> & img)
    {

        cv::Mat cvFrame;
        m_capture >> cvFrame;
        if (cvFrame.empty() || !cvFrame.data)
        {
            return FrameworkReturnCode::_ERROR_LOAD_IMAGE;
        }
        unsigned int w=cvFrame.rows;
        unsigned int h=cvFrame.cols;
        if(w!=m_parameters.resolution.width || h!=m_parameters.resolution.height)
            cv::resize(cvFrame, cvFrame, cv::Size((int)m_parameters.resolution.width,(int)m_parameters.resolution.height), 0, 0);

        return SolAROpenCVHelper::convertToSolar(cvFrame,img);
    }

    FrameworkReturnCode SolARVideoAsCameraOpencv::start(){

        LOG_INFO(" SolARVideoAsCameraOpencv::setParameters");
        if(m_capture.isOpened())
        {
            m_capture.release();
        }
        m_capture = cv::VideoCapture( m_videoPath);
        if (m_capture.isOpened())
        {
            if (m_is_resolution_set)
            {
                m_capture.set(cv::CAP_PROP_FRAME_WIDTH, m_parameters.resolution.width );
                m_capture.set(cv::CAP_PROP_FRAME_HEIGHT, m_parameters.resolution.height );
            }
            return FrameworkReturnCode::_SUCCESS;
        }
        else
        {
            LOG_ERROR("Cannot open video file {]", m_videoPath);
            return FrameworkReturnCode::_ERROR_;
        }
    }

    }
    }
}

