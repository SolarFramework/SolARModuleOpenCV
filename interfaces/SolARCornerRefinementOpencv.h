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

#ifndef SOLARCORNERREFINEMENTOPENCV_H
#define SOLARCORNERREFINEMENTOPENCV_H

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include "api/features/ICornerRefinement.h"
#include "opencv2/core.hpp"
#include <vector>

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARCornerRefinementOpencv
* @brief <B>Refine the corner locations.</B>
* <TT>UUID: ddae46ca-1657-4301-a87d-f2dcfa6265d0</TT>
*
*/

class SOLAROPENCV_EXPORT_API SolARCornerRefinementOpencv : public org::bcom::xpcf::ConfigurableBase,
	public api::features::ICornerRefinement
{
public:
	/// @brief SolARCornerRefinementOpencv constructor
	SolARCornerRefinementOpencv();

	/// @brief SolARCornerRefinementOpencv default destructor
	~SolARCornerRefinementOpencv() = default;

	/// @brief This method refines the corner locations
	/// @param[in] image Input image on which we are extracting keypoints.
	/// @param[in,out] corners Initial coordinates of the input corners and refined coordinates provided for output.
	void refine(const SRef<datastructure::Image> image, std::vector<datastructure::Point2Df> & corners) override;

	org::bcom::xpcf::XPCFErrorCode onConfigured() override final;

    void unloadComponent () override final;

private:
	uint32_t			m_nbMaxIters = 40;
	float				m_minAccuracy = 0.01;
	cv::TermCriteria	m_termcrit;
	uint32_t			m_winSize = 5;	
	cv::Size			m_subPixWinSize;
};

}
}
}

#endif // SOLARCORNERREFINEMENTOPENCV_H
