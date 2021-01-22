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

#include <opencv2/core.hpp>

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARKeylineDetectorOpencv
* @brief <B>Detects keylines in an image.</B>
* <TT>UUID: 2b901264-99a6-473d-861c-9c79c869ffd1</TT>
*
*/

	class SOLAROPENCV_EXPORT_API SolARKeylineDetectorOpencv : public org::bcom::xpcf::ConfigurableBase,
		public api::features::IKeylineDetector
	{
	public:
		SolARKeylineDetectorOpencv();
		~SolARKeylineDetectorOpencv() override;
		void unloadComponent() override final;

		org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

		org::bcom::xpcf::XPCFErrorCode setType(api::features::KeylineDetectorType type) override;

		api::features::KeylineDetectorType getType() override;

		org::bcom::xpcf::XPCFErrorCode initDetector() override;

		void detect(const SRef<datastructure::Image> image, std::vector<datastructure::Keyline> & keylines) override;

	private:
		// Computes different scale of the input image using gaussian blur and downscaling
		std::vector<cv::Mat> computeGaussianPyramids(const cv::Mat & opencvImage, int numOctaves, int scale);

		// The type of the selected detector type in string form
		std::string m_type{ "HOUGH" };
		// Specify a downscaling factor for the input image of the detector
		float m_imageRatio{ 1.f };
		// Image size reduction factor between octaves
		int m_reductionRatio{ 2 };
		// Number of image octaves computed to detect keylines at multiple scales
		int m_numOctaves{ 1 };
		// Keep only keylines that have a pixel length above this threshold
		int m_minLineLength{ 0 };
        // Maximum allowed gap between points on the same line to link them
        int m_maxLineGap{ 10 };
        // Distance resolution of the accumulator in pixels
        int m_distanceResolution{ 1 };
        // Angle resolution of the accumulator in radians
        float m_angleResolution{ static_cast<float>(CV_PI) / 180.f };
		// Threshold parameters to filter out low confidence lines
		int m_threshold{ 40 };
};
}
}
}

#endif // SOLARKEYLINEDETECTOROPENCV_H