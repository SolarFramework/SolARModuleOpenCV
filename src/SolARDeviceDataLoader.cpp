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

#include "SolARDeviceDataLoader.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include "xpcf/core/helpers.h"
#include <boost/filesystem.hpp>

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDeviceDataLoader)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARDeviceDataLoader::SolARDeviceDataLoader(): ConfigurableBase(xpcf::toUUID<SolARDeviceDataLoader>())
{
    declareInterface<api::input::devices::IARDevice>(this);
    declareProperty<std::string>("calibrationFile", m_calibrationFile);
    declareProperty<std::string>("pathToData", m_pathToData);
    declareProperty<int>("delayTime", m_delayTime);
}

SolARDeviceDataLoader::~SolARDeviceDataLoader()
{
	stop();
}

org::bcom::xpcf::XPCFErrorCode SolARDeviceDataLoader::onConfigured()
{
    // Load multi camera calibration file
	if (!boost::filesystem::exists(m_calibrationFile))
	{
		LOG_WARNING("Camera Calibration file path not exist");
		return xpcf::XPCFErrorCode::_FAIL;
	}
	if (!datastructure::loadFromFile(m_camRigParameters, m_calibrationFile)) {
		LOG_ERROR("Cannot load calibration file");
		return xpcf::XPCFErrorCode::_FAIL;
	}
	m_nbCameras = m_camRigParameters.cameraParams.size();
	m_cameras.resize(m_nbCameras);
	m_poseFiles.resize(m_nbCameras);
    return xpcf::XPCFErrorCode::_SUCCESS;
}

FrameworkReturnCode SolARDeviceDataLoader::start()
{
    // Prepare loader for images, poses
    for (int id_camera = 0; id_camera < m_nbCameras; ++id_camera) {
        char index[4] = {};
        std::sprintf(index, "%03d", id_camera);
        // pose loader
        std::string pathToPose = m_pathToData + "/pose_" + index + ".txt";
        m_poseFiles[id_camera].open(pathToPose);
        if (!m_poseFiles[id_camera].is_open()) {
            LOG_ERROR("Cannot load pose file of camera {}", id_camera);
            return FrameworkReturnCode::_ERROR_;
        }
        // camera loader
        std::string pathToImages = m_pathToData + "/" + index + "/%08d.jpg";
        m_cameras[id_camera].open(pathToImages);
        if (!m_cameras[id_camera].isOpened()) {
            LOG_ERROR("Cannot open images directory {}", pathToImages);
            return FrameworkReturnCode::_ERROR_;
        }
    }

    // Prepare loader timestamps
    std::string pathToTimestamps = m_pathToData + "/timestamps.txt";
    m_timestampFile.open(pathToTimestamps);
    if (!m_timestampFile.is_open()) {
        LOG_ERROR("Cannot open timestamps file {}", pathToTimestamps);
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARDeviceDataLoader::stop()
{
	for (int id_camera = 0; id_camera < m_nbCameras; ++id_camera) {
		if (m_cameras[id_camera].isOpened())
			m_cameras[id_camera].release();
		if (m_poseFiles[id_camera].is_open())
			m_poseFiles[id_camera].close();
	}
	if (m_timestampFile.is_open()) {
		m_timestampFile.close();
	}
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARDeviceDataLoader::getData(std::vector<SRef<Image>>& images, std::vector<Transform3Df>& poses, std::chrono::system_clock::time_point & timestamp)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(m_delayTime));
    for (int id_camera = 0; id_camera < m_nbCameras; ++id_camera) {
        // load images
        cv::Mat cvFrame;
        m_cameras[id_camera] >> cvFrame;
        if (!cvFrame.data)
            return FrameworkReturnCode::_ERROR_LOAD_IMAGE;
        SRef<Image> img;
        SolAROpenCVHelper::convertToSolar(cvFrame, img);
        images.push_back(img);

        // load poses
        Transform3Df pose;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                m_poseFiles[id_camera] >> pose(i, j);
        poses.push_back(pose);
    }

    // load timestamp
    int64 duration;
    m_timestampFile >> duration;
    std::chrono::milliseconds dur(duration);
    timestamp =  std::chrono::time_point<std::chrono::system_clock>(dur);
    return FrameworkReturnCode::_SUCCESS;
}

const SolAR::datastructure::CameraRigParameters & SolARDeviceDataLoader::getCameraParameters() const
{
	return m_camRigParameters;
}

}
}
}

