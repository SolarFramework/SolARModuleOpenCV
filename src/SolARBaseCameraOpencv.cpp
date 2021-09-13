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
#include <boost/filesystem.hpp>

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARBaseCameraOpencv)

namespace SolAR {
using namespace datastructure;
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
        if (!boost::filesystem::exists(m_calibrationFile))
        {
            LOG_WARNING("Camera Calibration file path not exist");
			return xpcf::XPCFErrorCode::_SUCCESS;
        }
		if (!datastructure::loadFromFile(m_parameters, m_calibrationFile)) {
			LOG_ERROR("Cannot load calibration file");
			return xpcf::XPCFErrorCode::_FAIL;
		}      
		return xpcf::XPCFErrorCode::_SUCCESS;
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

     const CameraParameters & SolARBaseCameraOpencv::getParameters() const
     {
        return m_parameters;
     }

     Sizei SolARBaseCameraOpencv::getResolution() const
     {
         return m_parameters.resolution;
     }

    const CamCalibration & SolARBaseCameraOpencv::getIntrinsicsParameters() const
    {
        return m_parameters.intrinsic;
    }

    const CamDistortion & SolARBaseCameraOpencv::getDistortionParameters() const
    {
        return m_parameters.distortion;
    }

}
}
}

