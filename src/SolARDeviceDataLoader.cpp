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

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARDeviceDataLoader)

namespace SolAR {
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
		for (int id_camera = 0; id_camera < m_nbCameras; ++id_camera) {
			if (m_cameras[id_camera].isOpened())
				m_cameras[id_camera].release();
			if (m_poseFiles[id_camera].is_open())
				m_poseFiles[id_camera].close();
		}
		m_timestampFile.close();
    }

	org::bcom::xpcf::XPCFErrorCode SolARDeviceDataLoader::onConfigured()
	{
		// Load multi camera calibration file
		if (m_calibrationFile.empty())
		{
			LOG_ERROR("Camera Calibration file path is empty");
			return xpcf::_FAIL;
		}
		cv::FileStorage fs(m_calibrationFile, cv::FileStorage::READ);
		cv::Mat intrinsic_parameters;
		cv::Mat distortion_parameters;
		int width, height;
		std::string cameraName;
		CameraParameters camParams;

		if (fs.isOpened())
		{
			fs["NbCameras"] >> m_nbCameras;
			for (int i = 0; i < m_nbCameras; ++i)
			{
				auto params = fs[std::to_string(i)];
				params["name"] >> cameraName;
				params["image_width"] >> width;
				params["image_height"] >> height;
				params["camera_matrix"] >> intrinsic_parameters;
				params["distortion_coefficients"] >> distortion_parameters;

				m_cameraNames.push_back(cameraName);
				camParams.resolution.width = width;
				camParams.resolution.height = height;

				if (intrinsic_parameters.empty())
				{
					LOG_ERROR("No intrinsics found in calibration file");
					return xpcf::_FAIL;
				}

				if (intrinsic_parameters.rows == camParams.intrinsic.rows() && intrinsic_parameters.cols == camParams.intrinsic.cols())
					for (int i = 0; i < intrinsic_parameters.rows; i++)
						for (int j = 0; j < intrinsic_parameters.cols; j++)
							camParams.intrinsic(i, j) = (float)intrinsic_parameters.at<double>(i, j);
				else
				{
					LOG_ERROR("Camera Calibration should be a 3x3 Matrix");
					return xpcf::_FAIL;
				}

				if (distortion_parameters.empty())
				{
					LOG_ERROR("No distortion parameters found in calibration file");
					return xpcf::_FAIL;
				}

				if (distortion_parameters.rows == camParams.distortion.rows() && distortion_parameters.cols == camParams.distortion.cols())
					for (int i = 0; i < distortion_parameters.rows; i++)
						for (int j = 0; j < distortion_parameters.cols; j++)
							camParams.distortion(i, j) = distortion_parameters.at<double>(i, j);
				else
				{
					LOG_ERROR("Camera distortion matrix should be a 5x1 Matrix");
					return xpcf::_FAIL;
				}
				m_camParameters.push_back(camParams);
				LOG_DEBUG("Loaded {} intrinsics", i);
			}
			return xpcf::_SUCCESS;
		}
		else
		{
			LOG_ERROR("Cannot open camera calibration file");
			return xpcf::_FAIL;
		}		

		return xpcf::_SUCCESS;
	}

	FrameworkReturnCode SolARDeviceDataLoader::start()
	{
		// Prepare loader for images, poses
		m_poseFiles.resize(m_nbCameras);
		m_cameras.resize(m_nbCameras);
		for (int id_camera = 0; id_camera < m_nbCameras; ++id_camera) {			
			char index[3];
			std::sprintf(index, "%03d", id_camera);

			// pose loader
			std::string pathToPose = m_pathToData + "/pose_" + index + ".txt";
			m_poseFiles[id_camera].open(pathToPose);
			if (!m_poseFiles[id_camera].is_open()) {
				LOG_ERROR("Cannot load pose file of camera {}", id_camera);
				FrameworkReturnCode::_ERROR_;
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
		return FrameworkReturnCode();
	}

	int SolARDeviceDataLoader::getNbCameras()
	{
		return m_nbCameras;
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
		int64 time;
		m_timestampFile >> time;
		// Todo: convert timestamp to time_point
		auto now = std::chrono::system_clock::now();
		auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);

		auto value = now_ms.time_since_epoch();
		long duration = value.count();

		std::chrono::milliseconds dur(duration);

		std::chrono::time_point<std::chrono::system_clock> dt(dur);
		return FrameworkReturnCode();
	}

	const CameraParameters & SolARDeviceDataLoader::getParameters(const int & camera_id)
	{
		return m_camParameters[camera_id];
	}

	void SolARDeviceDataLoader::setParameters(const int & camera_id, const CameraParameters & parameters)
	{
		m_camParameters[camera_id] = parameters;
	}

}
}
}

