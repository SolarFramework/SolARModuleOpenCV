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

#ifndef SOLARCAMERACALIBRATIONOPENCV_H
#define SOLARCAMERACALIBRATIONOPENCV_H

#include <string>
#include "opencv2/videoio.hpp"
#include "ComponentBase.h"
#include "api/input/devices/ICameraCalibration.h"

#include "SolAROpencvAPI.h"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

class SOLAROPENCV_EXPORT_API SolARCameraCalibrationOpencv : public org::bcom::xpcf::ComponentBase,
        public api::input::devices::ICameraCalibration {
public:
    SolARCameraCalibrationOpencv();
    virtual ~SolARCameraCalibrationOpencv();

	bool calibrate(std::string&inputVideo, std::string&output);
	bool calibrate(int camera_id, std::string&output);
	bool setParameters(std::string&config_file);
    virtual void unloadComponent () override;

    XPCF_DECLARE_UUID("702a7f53-e5ec-45d2-887d-daa99a34a33c");

protected:
    cv::Size m_boardSize;
    cv::Size m_imageSize;
    cv::Mat m_camMatrix;
    cv::Mat m_camDistorsion;

    float m_squareSize;
    float m_aspectRatio;

    int m_nframes;
    int m_flags ;
    int m_delay;	
	
	enum ProcessMode
	{
		SOLAR_DETECT = 0,
		SOLAR_CAPTURE = 1,
		SOLAR_CALIBRATED = 2
	};

	virtual bool process(cv::VideoCapture&, std::string& );
};

}
}
}  // end of namespace Solar



#endif // SOLARCAMERACALIBRATIONOPENCV_H
