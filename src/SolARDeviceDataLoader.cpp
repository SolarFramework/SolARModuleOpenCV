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
#include "opencv2/core/eigen.hpp"
#include "xpcf/core/helpers.h"

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
        declareProperty<std::string>("rectificationFile", m_rectificationFile);
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
        if (m_timestampFile.is_open()) {
            m_timestampFile.close();
        }
    }

    org::bcom::xpcf::XPCFErrorCode SolARDeviceDataLoader::onConfigured()
    {
        // Load multi camera calibration file
		if (m_calibrationFile.empty()) {
			LOG_ERROR("Camera Calibration file path is empty");
			return xpcf::XPCFErrorCode::_FAIL;
		}
        cv::FileStorage fs(m_calibrationFile, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            LOG_ERROR("Cannot open camera calibration file: {}", m_calibrationFile);
            return xpcf::XPCFErrorCode::_FAIL;
        }        

        fs["NbCameras"] >> m_nbCameras;
        if (m_nbCameras > 0) {
            m_poseFiles.resize(m_nbCameras);
            m_cameras.resize(m_nbCameras);
        }
        for (int i = 0; i < m_nbCameras; ++i)
        {
			cv::Mat intrinsic_parameters;
			cv::Mat distortion_parameters;
			int width, height;
			std::string cameraName;
			CameraParameters camParams;
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
                return xpcf::XPCFErrorCode::_FAIL;
            }
			cv::cv2eigen(intrinsic_parameters, camParams.intrinsic);
            
            if (distortion_parameters.empty())
            {
                LOG_ERROR("No distortion parameters found in calibration file");
                return xpcf::XPCFErrorCode::_FAIL;
            }
			cv::cv2eigen(distortion_parameters, camParams.distortion);

            m_camParameters.push_back(camParams);
            LOG_DEBUG("Loaded {} intrinsics", i);
        }
		fs.release();

		// Load rectification parameters
		if (m_rectificationFile.empty()) {
			LOG_WARNING("Rectification file path is empty");
			return xpcf::XPCFErrorCode::_SUCCESS;
		}
		fs.open(m_rectificationFile, cv::FileStorage::READ);
		if (!fs.isOpened()) {
			LOG_WARNING("Cannot open rectification file: {}", m_rectificationFile);
			return xpcf::XPCFErrorCode::_SUCCESS;
		}
		auto readNode = [](const cv::FileNode& node, uint32_t& id, RectificationParameters& rectParam) {
			int tmpId;
			node["Index"] >> tmpId;
			id = (uint32_t)tmpId;
			cv::Mat rot, proj;
			node["Rotation"] >> rot;
			node["Projection"] >> proj;
			cv::cv2eigen(rot, rectParam.rotation);
			cv::cv2eigen(proj, rectParam.projection);
			LOG_DEBUG("Camera {}", id);
			LOG_DEBUG("Rotation \n{}", rectParam.rotation.matrix());
			LOG_DEBUG("Projection \n{}", rectParam.projection.matrix());
		};

		fs["NbRectifications"] >> m_nbRect;
		for (int i = 0; i < m_nbRect; ++i){
			LOG_DEBUG("Rectification {}:", i);
			uint32_t id1, id2;
			std::vector<RectificationParameters> rectParams(2);
			auto params = fs["Rectification " + std::to_string(i)];
			auto camParams1 = params["Camera1"];
			auto camParams2 = params["Camera2"];
			readNode(camParams1, id1, rectParams[0]);
			readNode(camParams2, id2, rectParams[1]);
			if ((id1 >= m_nbCameras) || (id2 >= m_nbCameras)) {
				LOG_ERROR("No camera id existed for this rectification");
				return xpcf::XPCFErrorCode::_FAIL;
			}
			rectParams[0].camParams = m_camParameters[id1];
			rectParams[1].camParams = m_camParameters[id2];
			float fb, baseline;
			StereoType type;
			if (rectParams[1].projection(0, 3) != 0) {
				fb = std::abs(rectParams[1].projection(0, 3));				
				type = StereoType::Horizontal;
			}
			else {
				fb = std::abs(rectParams[1].projection(1, 3));
				type = StereoType::Vertical;
			}
			baseline = fb / rectParams[1].projection(0, 0);
			for (int j = 0; j < 2; ++j) {
				rectParams[j].baseline = baseline;
				rectParams[j].fb = fb;
				rectParams[j].type = type;
			}
			m_rectParams[std::make_pair(id1, id2)] = rectParams;
		}
		fs.release();
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
        int64 duration;
        m_timestampFile >> duration;
        std::chrono::milliseconds dur(duration);
        timestamp =  std::chrono::time_point<std::chrono::system_clock>(dur);
        return FrameworkReturnCode::_SUCCESS;
    }

    const CameraParameters & SolARDeviceDataLoader::getParameters(const int & camera_id) const
    {
        return m_camParameters[camera_id];
    }

	FrameworkReturnCode SolARDeviceDataLoader::getRectificationParameters(const std::pair<uint32_t, uint32_t>& pairCameraIds, std::vector<SolAR::datastructure::RectificationParameters>& rectParams) const
	{
		auto it = m_rectParams.find(pairCameraIds);
		if (it == m_rectParams.end())
			return FrameworkReturnCode::_ERROR_;
		else
			rectParams = it->second;
		return FrameworkReturnCode::_SUCCESS;
	}

    }
    }
}

