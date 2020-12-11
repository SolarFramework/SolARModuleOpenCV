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


#ifndef SOLARMAPFUSIONOPENCV_H
#define SOLARMAPFUSIONOPENCV_H

#include "xpcf/component/ConfigurableBase.h"
#include "SolAROpencvAPI.h"
#include "api/solver/map/IMapFusion.h"
#include "api/geom/I3DTransform.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/solver/pose/I3DTransformSACFinderFrom3D3D.h"

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

namespace SolAR {
namespace MODULES {
namespace OPENCV {

/**
* @class SolARMapFusionOpencv
* @brief <B>Merge local map or floating map in the global map.</B>
* <TT>UUID: bc661909-0185-40a4-a5e6-e52280e7b338</TT>
*
*/

class SOLAROPENCV_EXPORT_API SolARMapFusionOpencv : public org::bcom::xpcf::ConfigurableBase,
	public api::solver::map::IMapFusion {
public:
	///@brief SolARMapFusionOpencv constructor.
	SolARMapFusionOpencv();
	///@brief SolARMapFusionOpencv destructor.
	~SolARMapFusionOpencv() override;

	/// @brief Merge a map in the global map. The map can be a local map (know transformation to the global map) or a floating map.
	/// @param[in,out] map: local map or floating map to merge
	/// @param[in,out] globalMap: the global map
	/// @param[in,out] transform: the transformation to the global map (null for floating map). It can be refined by fusion process.
	/// @param[out] nbMatches: the number of matched cloud points.
	/// @param[error] error: the error of fusion process that is the mean of error distances of the matched cloud points.
	/// @return FrameworkReturnCode::_SUCCESS_ if the fusion succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode merge(SRef<api::solver::map::IMapper> &map,
							SRef<api::solver::map::IMapper> &globalMap,
							datastructure::Transform3Df &transform,
							uint32_t &nbMatches,
							float &error) override;

	/// @brief Merge a map in the global map. The map can be a local map (know transformation to the global map) or a floating map.
	/// @param[in,out] map: local map or floating map to merge
	/// @param[in,out] globalMap: the global map	
	/// @param[in,out] transform: the transformation to the global map (null for floating map). It can be refined by fusion process.
	/// @param[in] cpOverlapIndices : pairs of detected overlap cloud points indices of floating map and global map.
	/// @param[in] isRefineTransform : refine the 3D transformation if it's true.
	/// @return FrameworkReturnCode::_SUCCESS_ if the fusion succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode merge(SRef<api::solver::map::IMapper> &map,
							SRef<api::solver::map::IMapper> &globalMap,
							datastructure::Transform3Df &transform,
							const std::vector<std::pair<uint32_t, uint32_t>>&cpOverlapIndices,
							const bool &isRefineTransform = false) override;

	void unloadComponent() override final;

private:
	/// @brief fuse a map into the global map.
	/// @param[in] cpOverlapIndices : pairs of detected overlap cloud points indices of floating map and global map.
	/// @param[in,out] map: local map or floating map to merge
	/// @param[in,out] globalMap: the global map		
	void fuseMap(const std::vector<std::pair<uint32_t, uint32_t>>&cpOverlapIndices, SRef<api::solver::map::IMapper> &map, SRef<api::solver::map::IMapper> &globalMap);

private:
    float													m_radius = 0.3f;
	SRef<api::geom::I3DTransform>							m_transform3D;
	SRef<api::features::IDescriptorMatcher>					m_matcher;
	SRef<api::solver::pose::I3DTransformSACFinderFrom3D3D>	m_estimator3D;
};

}
}
}
#endif // SOLARMAPFUSIONOPENCV_H
