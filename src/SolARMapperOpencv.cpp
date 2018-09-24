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

#include "SolARMapperOpencv.h"

namespace xpcf  = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARMapperOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

    SolARMapperOpencv::SolARMapperOpencv():ComponentBase(xpcf::toUUID<SolARMapperOpencv>())
    {
        addInterface<IMapper>(this);
        m_map = xpcf::utils::make_shared<Map>() ;
    }

    void SolARMapperOpencv::addMatches(const std::pair<int,int>&working_views,
                    const std::vector<DescriptorMatch>& existingPointsMatches,
                    const std::vector<DescriptorMatch>& newPointsMatches)
    {
        m_gmatches[std::make_pair(working_views.first, working_views.second)] = newPointsMatches;

        // Update the visibility map for cloud points that are visible by both keyframes (match and have a 3D correspondent)
        SRef<std::vector<SRef<CloudPoint>>> pointCloud = m_map->getPointCloud();
        for (int i = 0; i < existingPointsMatches.size(); i++)
        {
            for (int j = 0; j < pointCloud->size(); j++)
            {
                if ((*pointCloud)[j]->m_visibility[working_views.first] == existingPointsMatches[i].getIndexInDescriptorA())
                {
                    (*pointCloud)[j]->m_visibility[working_views.second] = existingPointsMatches[i].getIndexInDescriptorB();
                    break;
                }
            }
        }
    }

    SRef<Map> SolARMapperOpencv::getMap()
    {
        return m_map ;
    }

    FrameworkReturnCode SolARMapperOpencv::update (SRef<Map> map,
                                                   SRef<Keyframe> newKeyframe,
                                                   const std::vector<SRef<CloudPoint>>& newCloud,
                                                   const std::vector<DescriptorMatch>& newPointsMatches,
                                                   const std::vector<DescriptorMatch>& existingPointsMatches)
    {
        if (m_kframes.size() == 0)
        {
            if (newCloud.size() != 0 || newPointsMatches.size() != 0 || existingPointsMatches.size() != 0)
            {
                LOG_WARNING("For the first update of the Mapper, only the first keyframe is required");
            }
            m_kframes.push_back(newKeyframe);
            map = m_map;
            return FrameworkReturnCode::_SUCCESS;
        }
        if (m_kframes.size() == 1)
        {
            if (existingPointsMatches.size() != 0)
                LOG_WARNING("For the second update of the Mapper, not need of existing");

            newKeyframe->getReferenceKeyframe()->addVisibleMapPoints(newCloud);
            newKeyframe->addVisibleMapPoints(newCloud);
            m_gmatches[std::make_pair(newKeyframe->getReferenceKeyframe()->m_idx, newKeyframe->m_idx)] = newPointsMatches;
            m_map->addCloudPoints(newCloud) ;
            return FrameworkReturnCode::_SUCCESS;
        }

        // A point cloud exist and we need to check if matches have already a corresponding 3D points
        std::vector<SRef<CloudPoint>> previous_cloud;
        SRef<std::vector<SRef<CloudPoint>>> pointCloud = m_map->getPointCloud();
        for (int i = 0; i < existingPointsMatches.size(); i++)
        {
            for (int j = 0; j < pointCloud->size(); j++)
            {
                if ((*pointCloud)[j]->m_visibility[newKeyframe->getReferenceKeyframe()->m_idx] == existingPointsMatches[i].getIndexInDescriptorA())
                {
                    previous_cloud.push_back((*pointCloud)[j]);
                }
            }
        }
        newKeyframe->addVisibleMapPoints(previous_cloud);

        // Add the 3D points that have just been triangulated
        newKeyframe->addVisibleMapPoints(newCloud);
        addMatches(std::make_pair(newKeyframe->m_idx,newKeyframe->getReferenceKeyframe()->m_idx),existingPointsMatches, newPointsMatches);
        m_map->addCloudPoints(newCloud);
        return FrameworkReturnCode::_SUCCESS;
    }
}
}
}
