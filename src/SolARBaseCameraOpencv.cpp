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

#include "SolARBaseCameraOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARBaseCameraOpencv)

namespace SolAR {
namespace MODULES {
namespace OPENCV {

    SolARBaseCameraOpencv::SolARBaseCameraOpencv(const org::bcom::xpcf::uuids::uuid & uuid):ConfigurableBase(uuid)
    {
        declareInterface<api::input::devices::ICamera>(this);
        declareProperty("calibrationFile", m_calibrationFile);
        m_is_resolution_set = false;
        m_parameters.distortion = CamDistortion::Zero();
        m_parameters.intrinsic = CamCalibration::Identity();
    }

    SolARBaseCameraOpencv::~SolARBaseCameraOpencv()
    {
        if (m_capture.isOpened())
        {
            m_capture.release();
        }
    }

    xpcf::XPCFErrorCode SolARBaseCameraOpencv::onConfigured()
    {
        LOG_DEBUG(" SolARBaseCameraOpencv onConfigured");
        if (m_calibrationFile.empty())
        {
            LOG_ERROR("Camera Calibration file path is empty");
            return xpcf::_FAIL;
        }
        cv::FileStorage fs(m_calibrationFile, cv::FileStorage::READ);
        cv::Mat intrinsic_parameters;
        cv::Mat distortion_parameters;

        if (fs.isOpened())
        {
            int width, height;
            fs["image_width"] >> width;
            fs["image_height"] >> height;
            fs["camera_matrix"] >> intrinsic_parameters;
            fs["distortion_coefficients"] >> distortion_parameters;

            m_parameters.resolution.width = width;
            m_parameters.resolution.height = height;
            m_is_resolution_set = true;

            if (intrinsic_parameters.empty())
            {
                LOG_ERROR ("SolARBaseCameraOpencv::loadCameraParameters: Use the landmark camera_matrix to define the intrinsic matrix in the .yml camera calibration file")
                return xpcf::_FAIL;
            }

            if (intrinsic_parameters.rows == m_parameters.intrinsic.rows() && intrinsic_parameters.cols == m_parameters.intrinsic.cols())
                for (int i = 0; i < intrinsic_parameters.rows; i++)
                    for (int j = 0; j < intrinsic_parameters.cols; j++)
                        m_parameters.intrinsic(i,j) = (float)intrinsic_parameters.at<double>(i,j);
            else
            {
                LOG_ERROR("SolARBaseCameraOpencv::loadCameraParameters: Camera Calibration should be a 3x3 Matrix")
                return xpcf::_FAIL;
            }

            if (distortion_parameters.empty())
            {
                LOG_ERROR("SolARBaseCameraOpencv::loadCameraParameters: Use the landmark distortion_coefficients to define the distortion vector in the .yml camera calibration file")
                return xpcf::_FAIL;
            }

            if (distortion_parameters.rows == m_parameters.distortion.rows() && distortion_parameters.cols == m_parameters.distortion.cols())
                for (int i = 0; i < distortion_parameters.rows; i++)
                    for (int j = 0; j < distortion_parameters.cols; j++)
                        m_parameters.distortion(i,j) = distortion_parameters.at<double>(i,j);
            else
            {
                LOG_ERROR("SolARBaseCameraOpencv::loadCameraParameters: Camera distortion matrix should be a 5x1 Matrix")
                return xpcf::_FAIL;
            }
            return xpcf::_SUCCESS;
        }
        else
        {
            LOG_ERROR("SolARBaseCameraOpencv::loadCameraParameters: Cannot open camera calibration file ")
            return xpcf::_FAIL;
        }
    }

    void SolARBaseCameraOpencv::setResolution(const Sizei & resolution)
    {
        m_parameters.resolution = resolution;
        m_is_resolution_set = true;
    }

    FrameworkReturnCode SolARBaseCameraOpencv::stop()
    {
        if(m_capture.isOpened())
        {
            m_capture.release();
        }
        return FrameworkReturnCode::_SUCCESS;
    }

    void SolARBaseCameraOpencv::setIntrinsicParameters(const CamCalibration & intrinsic_parameters){
        m_parameters.intrinsic = intrinsic_parameters;
    }

     void SolARBaseCameraOpencv::setDistortionParameters(const CamDistortion & distortion_parameters){
        m_parameters.distortion = distortion_parameters;
     }


     void SolARBaseCameraOpencv::setParameters(const CameraParameters & parameters)
     {
        m_parameters = parameters;
     }

     const CameraParameters & SolARBaseCameraOpencv::getParameters()
     {
        return m_parameters;
     }

     Sizei SolARBaseCameraOpencv::getResolution()
     {
         return m_parameters.resolution;
     }

    const CamCalibration & SolARBaseCameraOpencv::getIntrinsicsParameters(){
        return m_parameters.intrinsic;
    }

    const CamDistortion & SolARBaseCameraOpencv::getDistortionParameters(){
        return m_parameters.distortion;
    }

}
}
}

