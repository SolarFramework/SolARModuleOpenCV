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

#include "SolAR2D3DcorrespondencesFinderOpencv.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolAR2D3DCorrespondencesFinderOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {
SolAR2D3DCorrespondencesFinderOpencv::SolAR2D3DCorrespondencesFinderOpencv():ComponentBase(xpcf::toUUID<SolAR2D3DCorrespondencesFinderOpencv>())
{
    declareInterface<api::solver::pose::I2D3DCorrespondencesFinder>(this);
	declareInjectable<IPointCloudManager>(m_pointCloudManager);
    LOG_DEBUG("SolAR2D3DCorrespondencesFinder constructor");
}

FrameworkReturnCode SolAR2D3DCorrespondencesFinderOpencv::find(const SRef<Frame> lastFrame, const SRef<Frame> currentFrame, const std::vector<DescriptorMatch>& current_matches, std::vector<Point3Df>& shared_3dpoint, std::vector<Point2Df>& shared_2dpoint, std::vector<DescriptorMatch>& found_matches, std::vector<DescriptorMatch>& remaining_matches)
{
	const std::map<uint32_t, uint32_t> &mapVisibility = lastFrame->getVisibility();
	const std::vector<Keypoint> &current_kpoints = currentFrame->getKeypoints();

	for (int j = 0; j < current_matches.size(); ++j) {
		SRef<CloudPoint> point3D;
		std::map<unsigned int, unsigned int>::const_iterator it_cp = mapVisibility.find(current_matches[j].getIndexInDescriptorA());
		if ((it_cp != mapVisibility.end()) && (m_pointCloudManager->getPoint(it_cp->second, point3D) == FrameworkReturnCode::_SUCCESS)) {
			shared_3dpoint.push_back(Point3Df(point3D->getX(), point3D->getY(), point3D->getZ()));
			shared_2dpoint.push_back(Point2Df(current_kpoints[current_matches[j].getIndexInDescriptorB()].getX(),
				current_kpoints[current_matches[j].getIndexInDescriptorB()].getY()));
			found_matches.push_back(current_matches[j]);
		}
		else {
			remaining_matches.push_back(current_matches[j]);
		}
	}

	return FrameworkReturnCode::_SUCCESS;
}

}
}
}
