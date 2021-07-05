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

#ifndef SOLARDEVICEDATALOADER_H
#define SOLARDEVICEDATALOADER_H

#include "SolAROpencvAPI.h"
#include <vector>
#include <string>
#include <fstream>
#include "api/input/devices/IARDevice.h"
#include "xpcf/component/ConfigurableBase.h"
#include "opencv2/opencv.hpp"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
 * @class SolARDeviceDataLoader
 * @brief <B>Load AR device data including images, poses, timestamp.</B>
 * <TT>UUID: 4b5576c1-4c44-4835-a405-c8de2d4f85b0</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ calibrationFile,
 *                          ,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ pathToData,
 *                          ,
 *                          @SolARComponentPropertyDescString{ "" }}
 * @SolARComponentProperty{ delayTime,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 0 }}
 * @SolARComponentPropertiesEnd
 * 
 */

class SOLAROPENCV_EXPORT_API SolARDeviceDataLoader : public org::bcom::xpcf::ConfigurableBase,
	public api::input::devices::IARDevice {
public:
	/// @brief Specify the IBuiltInSLAM constructor class
	SolARDeviceDataLoader();

	/// @brief Specify the IBuiltInSLAM destructor class
	~SolARDeviceDataLoader();

	org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

	void unloadComponent() override final;

	/// @brief Start the connection to the device for sensors data streaming.
	/// @return FrameworkReturnCode::_SUCCESS if successful, eiher FrameworkReturnCode::_ERROR_.
	FrameworkReturnCode start() override;

	/// @brief Stop the connection to the device.
	/// @return FrameworkReturnCode::_SUCCESS if successful, eiher FrameworkReturnCode::_ERROR_.
	FrameworkReturnCode stop() override;

	/// @brief Get number of cameras of the device.
	/// @return the number of cameras.
	int getNbCameras() override;

	/// @brief Retrieve a set of images and their associated poses from the sensors as well as timestamp.
	/// @param[out] images: the captured images.
	/// @param[out] poses: the associated poses.
	/// @param[out] timestamp: the timestamp.
	/// @return FrameworkReturnCode to track successful or failing event.
	FrameworkReturnCode getData(std::vector<SRef<datastructure::Image>> & images, std::vector<datastructure::Transform3Df> & poses, std::chrono::system_clock::time_point &timestamp) override;

	/// @brief Get the distortion and intrinsic camera parameters
	/// @param[in] camera_id: The id of the camera.
	/// @return the camera parameters
    const datastructure::CameraParameters & getParameters(const int & camera_id) const override;

	/// @brief Get the rectification parameters of a stereo camera
	/// @param[in] pairCameraIds The pair ids of the stereo camera.
	/// @param[out] rectParams The vector of rectification parameters of the stereo camera
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode getRectificationParameters(const std::pair<uint32_t, uint32_t>& pairCameraIds,
												   std::vector<SolAR::datastructure::RectificationParameters>& rectParams) const override;

 private:
	 int											m_nbCameras, m_nbRect;
	 std::string									m_calibrationFile;
	 std::string									m_rectificationFile;
	 std::string									m_pathToData;
	 std::vector<datastructure::CameraParameters>	m_camParameters;
	 std::vector<std::string>						m_cameraNames;
	 std::vector<cv::VideoCapture>					m_cameras;
	 std::vector<std::ifstream>						m_poseFiles;
	 std::ifstream									m_timestampFile;
	 int											m_delayTime = 0;
	 std::map< std::pair<uint32_t, uint32_t>, std::vector<SolAR::datastructure::RectificationParameters>> m_rectParams;
};

}
}
}

#endif
