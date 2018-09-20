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

                SolARMapperOpencv::SolARMapperOpencv():ConfigurableBase(xpcf::toUUID<SolARMapperOpencv>())
                {
                    addInterface<IMapper>(this);
                    m_map = xpcf::utils::make_shared<Map>() ;
                    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
                    params->wrapInteger("minNbMatchesIsKeyframe", m_minNbMatchesIsKeyframe);
                    params->wrapFloat("minMeanDistanceIsKeyframe", m_minMeanDistanceIsKeyframe);
                }

                void SolARMapperOpencv::addNewKeyFrame(const SRef<Frame> & frame, SRef<Keyframe>& newKeyframe) {
                    newKeyframe = xpcf::utils::make_shared<Keyframe>(frame->getView(), frame->getDescriptors(), m_kframes.size(), frame->m_pose, frame->getKeyPoints());
                    m_kframes.push_back(newKeyframe);
                }

                void  SolARMapperOpencv::removeKeyframe(const SRef<Keyframe>&old_kframe){
                }
                void SolARMapperOpencv::addMatches(const std::pair<int,int>&working_views,
                                                   const std::vector<DescriptorMatch>& found_matches,
                                                   const std::vector<DescriptorMatch>& new_matches){
                    m_gmatches[std::make_pair(working_views.first, working_views.second)] = new_matches;

                    // Update the visibility map for cloud points that are visible by both keyframes (match and have a 3D correspondent)
                    SRef<std::vector<SRef<CloudPoint>>> pointCloud = m_map->getPointCloud();
                    for (int i = 0; i < found_matches.size(); i++)
                    {
                        for (int j = 0; j < pointCloud->size(); j++)
                        {
                            if ((*pointCloud)[j]->m_visibility[working_views.first] == found_matches[i].getIndexInDescriptorA())
                            {
                                (*pointCloud)[j]->m_visibility[working_views.second] = found_matches[i].getIndexInDescriptorB();
                                break;
                            }
                        }
                    }
                }

                bool SolARMapperOpencv::initMap(SRef<Keyframe>&kframe_t0,
                                                SRef<Keyframe>&kframe_t1,
                                                std::vector<SRef<CloudPoint>>&initCloud,
                                                std::vector<DescriptorMatch>&matches){

                    kframe_t0->addVisibleMapPoints(initCloud);
                    kframe_t1->addVisibleMapPoints(initCloud);
                    m_gmatches[std::make_pair(kframe_t0->m_idx, kframe_t1->m_idx)] = matches;
                    LOG_DEBUG("init map with {} points", initCloud.size());
                    m_map->addCloudPoints(initCloud) ;
                    return true;
                }


                bool SolARMapperOpencv::updateMap(const SRef<Keyframe>& new_kframe,
                                                  const std::vector<DescriptorMatch>& found_matches,
                                                  const std::vector<DescriptorMatch>& new_matches,
                                                  const std::vector<SRef<CloudPoint>>& newCloud){
                    // Add the 3D points already visible from the previous keyframe
                    std::vector<SRef<CloudPoint>> previous_cloud;
                    SRef<std::vector<SRef<CloudPoint>>> pointCloud = m_map->getPointCloud();
                    for (int i = 0; i < found_matches.size(); i++)
                    {
                        for (int j = 0; j < pointCloud->size(); j++)
                        {
                            if ((*pointCloud)[j]->m_visibility[new_kframe->m_idx -1] == found_matches[i].getIndexInDescriptorA())
                            {
                                previous_cloud.push_back((*pointCloud)[j]);
                            }
                        }
                    }
                    new_kframe->addVisibleMapPoints(previous_cloud);

                    // Add the 3D points that have just been triangulated
                    new_kframe->addVisibleMapPoints(newCloud);
                    std::pair<int,int>working_views = std::make_pair(new_kframe->m_idx-1,new_kframe->m_idx);
                    addMatches(working_views,found_matches, new_matches);
                    m_map->addCloudPoints(newCloud);
                    return true;
                }

                SRef<Keyframe> SolARMapperOpencv::getCurrentKeyframe(int idx){
                    if(idx < m_kframes.size()){
                         return m_kframes[idx];
                    }
                    else{
                        LOG_ERROR("error, can't retrieve last keyframe, id must be lower than keyframes size");
                    }
                }
                void SolARMapperOpencv::associateReferenceKeyFrameToFrame(SRef<Frame> frame)
                {
                    // for now : choose last one
                    frame->setReferenceKeyFrame((m_kframes[m_kframes.size()-1]));
                }

                SRef<Map> SolARMapperOpencv::getMap()
                {
                    return m_map ;
                }

                bool SolARMapperOpencv::isKeyFrameCandidate(const std::vector<SRef<Keypoint>>& keypointsRef, const std::vector<SRef<Keypoint>>& keypointsCurrent, const std::vector<DescriptorMatch>& matches, const unsigned int imageWidth)
                {
                    if (matches.size() < m_minNbMatchesIsKeyframe)
                        return false;

                    double totalMatchesDist = 0.0;
                    for (int i = 0; i < matches.size(); i++)
                    {
                        SRef<Keypoint> keypointRef = keypointsRef[matches[i].getIndexInDescriptorA()];
                        SRef<Keypoint> keypointCurrent = keypointsCurrent[matches[i].getIndexInDescriptorB()];

                        totalMatchesDist+=((*keypointRef)-(*keypointCurrent)).norm()/imageWidth;
                    }
                    return (totalMatchesDist/matches.size()>m_minMeanDistanceIsKeyframe);

                    /*
                    bool enoughFrameSinceKf = ( frame->getNumberOfFramesSinceLastKeyFrame() >20) ; // more than 20 frames since last key frame
                    bool enoughTrackPoints = (frame->getCommonMapPointsWithReferenceKeyFrame().size() > 20) ; // enough map points seen in frame
                    float redudancyValue = ( (float) frame->getCommonMapPointsWithReferenceKeyFrame().size() / (float) frame->getReferenceKeyFrame()->getVisibleMapPoints().size()) ;  // percent of track kref map points seen in frame
                    bool notRedundancy = (redudancyValue < 0.9) ;

                    Transform3Df poseFrame = frame->m_pose ;
                    Transform3Df poseKeyFrame = frame->getReferenceKeyFrame()->m_pose ;

                    Vector3f tFrame = poseFrame.translation() ;
                    Vector3f tkeyFrame = poseKeyFrame.translation() ;
                    Vector3f difPos = tFrame - tkeyFrame ;
                    float translationDistance = difPos.norm() ;

                    Quaternionf rFrame(poseFrame.rotation()) ;
                    Quaternionf rkeyFrame(poseKeyFrame.rotation()) ;
                    float rotationDistance = rFrame.angularDistance(rkeyFrame) ;

                    LOG_DEBUG(" Translation Distance {}, Angular Distance {}", translationDistance, rotationDistance);


                    // to do : add  these threshold as parameters somewhere
					bool enoughDistance = ((translationDistance > 0.15) || (rotationDistance > 3.14/8.0)) ;
					bool conditionAddKeyFrame = (enoughFrameSinceKf && enoughTrackPoints && notRedundancy && enoughDistance ) ;
                    // For now : 20 frames and enough track points only!!
					if (conditionAddKeyFrame)
                    {
                        LOG_DEBUG(" Add new Key Frame \nNb Frames {} \nTrack Points {} \nRedundancy {} \nTranslation Distance {} \nRotation Distance {}", frame->getNumberOfFramesSinceLastKeyFrame(), frame->getCommonMapPointsWithReferenceKeyFrame().size(), redudancyValue, translationDistance, rotationDistance);
                        return m_kframes.size(); // index of key frame matches with array length

                    }
                    */
                    return -1 ;

                }


           }
        }
}
