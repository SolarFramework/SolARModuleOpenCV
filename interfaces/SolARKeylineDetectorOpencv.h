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

#ifndef SOLARKEYLINEDETECTOROPENCV_H
#define SOLARKEYLINEDETECTOROPENCV_H

#include "api/features/IKeylineDetector.h"

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"

namespace SolAR {
using namespace datastructure;
using namespace api::features;
namespace MODULES {
namespace OPENCV {

/**
* @class SolARKeylineDetectorOpenCV
* @brief <B>Detects keylines in an image.</B>
* <TT>UUID: be7c9a63-844e-42e2-8efb-e4848f94fbeb</TT>
*
*/

class SOLAROPENCV_EXPORT_API SolARKeylineDetectorOpencv : public org::bcom::xpcf::ConfigurableBase,
	public IKeylineDetector
{
public:
	SolARKeylineDetectorOpencv();
	~SolARKeylineDetectorOpencv() override;
	void unloadComponent() override final;

	org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

	void setType(KeylineDetectorType type) override;

	KeylineDetectorType getType() override;

	void detect(const SRef<Image> image, std::vector<Keyline> & keylines) override;

private:
	std::string m_type = "LSD";
	cv::Ptr<cv::LineSegmentDetector> m_detector;
	
	float m_imageRatio = 1.0f;

	int m_scale = 1;
	int m_numOctave = 4;
	
	int m_nbDescriptors = 10000;
};
}
}
}


#endif // SOLARKEYLINEDETECTOROPENCV_H
