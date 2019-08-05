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

                LOG_DEBUG("SolAR2D3DCorrespondencesFinder constructor");
            }

            //SolAR2D3DCorrespondencesFinderOpencv::~SolAR2D3DCorrespondencesFinderOpencv(){
            //}

            FrameworkReturnCode SolAR2D3DCorrespondencesFinderOpencv::find(const SRef<Keyframe> referenceKeyframe,
                                                                           const SRef<Frame> currentFrame,
                                                                           const std::vector<DescriptorMatch> & current_matches,
                                                                           std::vector<CloudPoint> & shared_mapPoint,
                                                                           std::vector<Point3Df> & shared_3dpoint,
                                                                           std::vector<Point2Df> & shared_2dpoint,
                                                                           std::vector<DescriptorMatch> & found_matches,
                                                                           std::vector<DescriptorMatch> & remaining_matches){


                 const std::map<unsigned int, CloudPoint> keyframeVisibility = referenceKeyframe->getVisibleMapPoints();
                 const std::vector<Keypoint> current_kpoints =  currentFrame->getKeypoints();
                 for (const auto & current_matche : current_matches)
                 {
                    auto it= keyframeVisibility.find(current_matche.getIndexInDescriptorA());
                    if (it != keyframeVisibility.end())
                    {
                            shared_mapPoint.emplace_back(it->second) ;
                            shared_3dpoint.emplace_back(it->second.x(),it->second.y(),it->second.z());
                            shared_2dpoint.emplace_back(current_kpoints[current_matche.getIndexInDescriptorB()].x(),
                                                                              current_kpoints[current_matche.getIndexInDescriptorB()].y());
                            found_matches.emplace_back(current_matche);
                    }
                    else
                    {
                        remaining_matches.emplace_back(current_matche);
                    }
                }

               // std::cout<<" point cloud size: "<<cloud.size()<<" shared: "<<shared_3dpoint.size()<<std::endl;
                return FrameworkReturnCode::_SUCCESS;
            }

			FrameworkReturnCode SolAR2D3DCorrespondencesFinderOpencv::find(	const SRef<Frame> lastFrame,
																			const SRef<Frame> currentFrame,
                                                                            const std::vector<DescriptorMatch> & current_matches,
                                                                            std::vector<CloudPoint> & shared_mapPoint,
                                                                            std::vector<Point3Df> & shared_3dpoint,
                                                                            std::vector<Point2Df> & shared_2dpoint,
																			std::vector<DescriptorMatch> & found_matches,
																			std::vector<DescriptorMatch> & remaining_matches) {

				const std::map<unsigned int, unsigned int> kpKeyframeVisibility = lastFrame->getVisibleKeypoints();
                const std::map<unsigned int, CloudPoint> frameVisibility = lastFrame->getReferenceKeyframe()->getVisibleMapPoints();
                const std::vector<Keypoint> current_kpoints = currentFrame->getKeypoints();
				std::map<unsigned int, unsigned int> newKpKeyframeVisibility;
				for (const auto & current_matche : current_matches)
				{
					auto it_kp = kpKeyframeVisibility.find(current_matche.getIndexInDescriptorA());
					if (it_kp != kpKeyframeVisibility.end()) {
						newKpKeyframeVisibility[current_matche.getIndexInDescriptorB()] = it_kp->second;
                        auto it_pcl = frameVisibility.find(it_kp->second);
						if (it_pcl != frameVisibility.end())
						{
                            shared_mapPoint.emplace_back(it_pcl->second);
                            shared_3dpoint.emplace_back(it_pcl->second.x(), it_pcl->second.y(), it_pcl->second.z());
                            shared_2dpoint.emplace_back(current_kpoints[current_matche.getIndexInDescriptorB()].x(),
                                current_kpoints[current_matche.getIndexInDescriptorB()].y());
                            found_matches.emplace_back(it_kp->second, current_matche.getIndexInDescriptorB(), current_matche.getMatchingScore());
						}
						else
						{
                            remaining_matches.emplace_back(it_kp->second, current_matche.getIndexInDescriptorB(), current_matche.getMatchingScore());
						}
					}
				}

				currentFrame->addVisibleKeypoints(newKpKeyframeVisibility);

				// std::cout<<" point cloud size: "<<cloud.size()<<" shared: "<<shared_3dpoint.size()<<std::endl;
				return FrameworkReturnCode::_SUCCESS;

			}
        }
    }
}
