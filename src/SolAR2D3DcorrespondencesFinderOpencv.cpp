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
																		   const SRef<Map> worldMap,
                                                                           std::vector<Point3Df> & shared_3dpoint,
                                                                           std::vector<Point2Df> & shared_2dpoint,
                                                                           std::vector<DescriptorMatch> & found_matches,
                                                                           std::vector<DescriptorMatch> & remaining_matches){


                 const std::map<unsigned int, unsigned int> keyframeVisibility = referenceKeyframe->getVisibleMapPoints();
                 const std::vector<Keypoint> current_kpoints =  currentFrame->getKeypoints();

				 std::vector<CloudPoint> cloudPoint = worldMap->getPointCloud();

                 for (int j = 0; j < current_matches.size(); ++j)
                 {
                    std::map<unsigned int, unsigned int>::const_iterator it= keyframeVisibility.find(current_matches[j].getIndexInDescriptorA());
                    if (it != keyframeVisibility.end())
                    {
                            shared_3dpoint.push_back(Point3Df(cloudPoint[it->second].getX(), cloudPoint[it->second].getY(), cloudPoint[it->second].getZ()));
                            shared_2dpoint.push_back(Point2Df(current_kpoints[current_matches[j].getIndexInDescriptorB()].getX(),
                                                                              current_kpoints[current_matches[j].getIndexInDescriptorB()].getY()));
                            found_matches.push_back(current_matches[j]);
                    }
                    else
                    {
                        remaining_matches.push_back(current_matches[j]);
                    }
                }

               // std::cout<<" point cloud size: "<<cloud.size()<<" shared: "<<shared_3dpoint.size()<<std::endl;
                return FrameworkReturnCode::_SUCCESS;
            }

			FrameworkReturnCode SolAR2D3DCorrespondencesFinderOpencv::find(	const SRef<Frame> lastFrame,
																			const SRef<Frame> currentFrame,
                                                                            const std::vector<DescriptorMatch> & current_matches,
																			const SRef<Map> worldMap,
                                                                            std::vector<Point3Df> & shared_3dpoint,
                                                                            std::vector<Point2Df> & shared_2dpoint,
																			std::vector<DescriptorMatch> & found_matches,
																			std::vector<DescriptorMatch> & remaining_matches) {

				const std::map<unsigned int, unsigned int> mapVisibility = lastFrame->getVisibleMapPoints();
                const std::map<unsigned int, unsigned int> cpVisibility = lastFrame->getReferenceKeyframe()->getVisibleMapPoints();
                const std::vector<Keypoint> current_kpoints = currentFrame->getKeypoints();

				std::vector<CloudPoint> cloudPoint = worldMap->getPointCloud();

				//std::map<unsigned int, unsigned int> newMapVisibility;
				for (int j = 0; j < current_matches.size(); ++j)
				{
					std::map<unsigned int, unsigned int>::const_iterator it_cp = mapVisibility.find(current_matches[j].getIndexInDescriptorA());
					if (it_cp != mapVisibility.end()) {
						//newMapVisibility[current_matches[j].getIndexInDescriptorB()] = it_cp->second;
                        shared_3dpoint.push_back(Point3Df(cloudPoint[it_cp->second].getX(), cloudPoint[it_cp->second].getY(), cloudPoint[it_cp->second].getZ()));
                        shared_2dpoint.push_back(Point2Df(current_kpoints[current_matches[j].getIndexInDescriptorB()].getX(),
                            current_kpoints[current_matches[j].getIndexInDescriptorB()].getY()));
						found_matches.push_back(DescriptorMatch(current_matches[j].getIndexInDescriptorA(), current_matches[j].getIndexInDescriptorB(), current_matches[j].getMatchingScore()));
					}
					else {
						remaining_matches.push_back(DescriptorMatch(current_matches[j].getIndexInDescriptorA(), current_matches[j].getIndexInDescriptorB(), current_matches[j].getMatchingScore()));
					}
				}

				//currentFrame->addVisibleMapPoints(newMapVisibility);

				// std::cout<<" point cloud size: "<<cloud.size()<<" shared: "<<shared_3dpoint.size()<<std::endl;
				return FrameworkReturnCode::_SUCCESS;

			}

			//FrameworkReturnCode SolAR2D3DCorrespondencesFinderOpencv::find(	const SRef<Frame> lastFrame,
			//																const SRef<Frame> currentFrame,
			//																const std::vector<DescriptorMatch> & current_matches,
			//																const SRef<Map> worldMap,
			//																std::vector<Point3Df> & shared_3dpoint,
			//																std::vector<Point2Df> & shared_2dpoint,
			//																std::vector<DescriptorMatch> & found_matches,
			//																std::vector<DescriptorMatch> & remaining_matches) {

			//	const std::map<unsigned int, unsigned int> kpKeyframeVisibility = lastFrame->getVisibleKeypoints();
			//	const std::map<unsigned int, unsigned int> cpVisibility = lastFrame->getReferenceKeyframe()->getVisibleMapPoints();
			//	const std::vector<Keypoint> current_kpoints = currentFrame->getKeypoints();

			//	std::vector<CloudPoint> cloudPoint = worldMap->getPointCloud();

			//	std::map<unsigned int, unsigned int> newKpKeyframeVisibility;
			//	for (int j = 0; j < current_matches.size(); ++j)
			//	{
			//		std::map<unsigned int, unsigned int>::const_iterator it_kp = kpKeyframeVisibility.find(current_matches[j].getIndexInDescriptorA());
			//		if (it_kp != kpKeyframeVisibility.end()) {
			//			newKpKeyframeVisibility[current_matches[j].getIndexInDescriptorB()] = it_kp->second;
			//			std::map<unsigned int, unsigned int>::const_iterator it_pcl = cpVisibility.find(it_kp->second);
			//			if (it_pcl != cpVisibility.end())
			//			{
			//				shared_3dpoint.push_back(Point3Df(cloudPoint[it_pcl->second].getX(), cloudPoint[it_pcl->second].getY(), cloudPoint[it_pcl->second].getZ()));
			//				shared_2dpoint.push_back(Point2Df(current_kpoints[current_matches[j].getIndexInDescriptorB()].getX(),
			//					current_kpoints[current_matches[j].getIndexInDescriptorB()].getY()));
			//				found_matches.push_back(DescriptorMatch(it_kp->second, current_matches[j].getIndexInDescriptorB(), current_matches[j].getMatchingScore()));
			//			}
			//			else
			//			{
			//				remaining_matches.push_back(DescriptorMatch(it_kp->second, current_matches[j].getIndexInDescriptorB(), current_matches[j].getMatchingScore()));
			//			}
			//		}
			//	}

			//	currentFrame->addVisibleKeypoints(newKpKeyframeVisibility);

			//	// std::cout<<" point cloud size: "<<cloud.size()<<" shared: "<<shared_3dpoint.size()<<std::endl;
			//	return FrameworkReturnCode::_SUCCESS;

			//}
        }
    }
}
