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

#include "SolARImagesAsCameraOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARImagesAsCameraOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

    SolARImagesAsCameraOpencv::SolARImagesAsCameraOpencv():SolARBaseCameraOpencv(xpcf::toUUID<SolARImagesAsCameraOpencv>())
    {
        declareInterface<api::input::devices::ICamera>(this);
        declareProperty("imagesDirectoryPath", m_ImagesDirectoryPath);
        declareProperty("delayTime", m_delayTime);
    }  

    FrameworkReturnCode SolARImagesAsCameraOpencv::getNextImage(SRef<Image> & img)
    {

        cv::Mat cvFrame;
		std::this_thread::sleep_for(std::chrono::milliseconds(m_delayTime));
        m_capture >> cvFrame;
        if(!cvFrame.data)
            return FrameworkReturnCode::_ERROR_LOAD_IMAGE;

        unsigned int w=cvFrame.cols;
        unsigned int h=cvFrame.rows;
        if (m_is_resolution_set && (w!=m_parameters.resolution.width || h!=m_parameters.resolution.height))
                cv::resize(cvFrame, cvFrame, cv::Size((int)m_parameters.resolution.width,(int)m_parameters.resolution.height), 0, 0);

        return SolAROpenCVHelper::convertToSolar(cvFrame,img);
    }

    FrameworkReturnCode SolARImagesAsCameraOpencv::start(){

        LOG_INFO(" SolARImagesAsCameraOpencv::setParameters");
        if(m_capture.isOpened())
        {
            m_capture.release();
        }
        m_capture = cv::VideoCapture( m_ImagesDirectoryPath);
        if (m_capture.isOpened())
        {
            if (m_is_resolution_set)
            {
                LOG_INFO("Camera using {}  *  {} resolution", m_parameters.resolution.width ,m_parameters.resolution.height)

                auto setResolution = [&]
                                     (cv::VideoCaptureProperties dimensionProp,
                                      uint32_t value,
                                      const std::string& dimensionName)
                {
                  bool setResDimOk = m_capture.set(dimensionProp, value);

                  if (!setResDimOk || m_capture.get(dimensionProp) != value)
                  {
                    LOG_WARNING("Cannot set camera {} to {}. Will fallback to cv::resize() each frame", dimensionName, value);
                    if (!setResDimOk)
                    {
                      LOG_WARNING( "Note: cv::VideoCapture::set() returned 'true'");
                    }
                  }
                };

                setResolution(cv::CAP_PROP_FRAME_WIDTH, m_parameters.resolution.width, "width");
                setResolution(cv::CAP_PROP_FRAME_HEIGHT, m_parameters.resolution.height, "height");

            }
            return FrameworkReturnCode::_SUCCESS;
        }
        else
        {
            LOG_ERROR("Cannot open images directory {}", m_ImagesDirectoryPath);
            return FrameworkReturnCode::_ERROR_;
        }
    }
}
}
}

