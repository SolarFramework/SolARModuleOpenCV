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

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolAR2D3DCorrespondencesFinderOpencv)

namespace SolAR {
    using namespace datastructure;
    namespace MODULES {
        namespace OPENCV {
            SolAR2D3DCorrespondencesFinderOpencv::SolAR2D3DCorrespondencesFinderOpencv():ComponentBase(xpcf::toUUID<SolAR2D3DCorrespondencesFinderOpencv>())
            {
                addInterface<api::solver::pose::I2D3DCorrespondencesFinder>(this);

                LOG_DEBUG("SolAR2D3DCorrespondencesFinder constructor");
            }

            //SolAR2D3DCorrespondencesFinderOpencv::~SolAR2D3DCorrespondencesFinderOpencv(){
            //}

            FrameworkReturnCode SolAR2D3DCorrespondencesFinderOpencv::find(const SRef<Keyframe> referenceKeyframe,
                                                                           const SRef<Frame> currentFrame,
                                                                           const std::vector<DescriptorMatch>&current_matches,
                                                                           std::vector<SRef<CloudPoint>>&shared_mapPoint,
                                                                           std::vector<SRef<Point3Df>>&shared_3dpoint,
                                                                           std::vector<SRef<Point2Df>>&shared_2dpoint,
                                                                           std::vector<DescriptorMatch> & found_matches,
                                                                           std::vector<DescriptorMatch> & remaining_matches){


                 const std::map<unsigned int, SRef<CloudPoint>> keyframeVisibility = referenceKeyframe->getVisibleMapPoints();
                 const std::vector<SRef<Keypoint>> current_kpoints =  currentFrame->getKeypoints();
                 for (int j = 0; j < current_matches.size(); ++j)
                 {
                    std::map<unsigned int, SRef<CloudPoint>>::const_iterator it= keyframeVisibility.find(current_matches[j].getIndexInDescriptorA());
                    if (it != keyframeVisibility.end())
                    {
                            shared_mapPoint.push_back(it->second) ;
                            shared_3dpoint.push_back(xpcf::utils::make_shared<Point3Df>(it->second->getX(),it->second->getY(),it->second->getZ()));
                            shared_2dpoint.push_back(xpcf::utils::make_shared<Point2Df>(current_kpoints[current_matches[j].getIndexInDescriptorB()]->getX(),
                                                                              current_kpoints[current_matches[j].getIndexInDescriptorB()]->getY()));
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
        }
    }
}
