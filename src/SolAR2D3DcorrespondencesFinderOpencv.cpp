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
#include "SolAROpenCVHelper.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "xpcf/component/ComponentBase.h"


#include <map>

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

            FrameworkReturnCode SolAR2D3DCorrespondencesFinderOpencv::find(const std::vector<SRef<CloudPoint>>&cloud,
                                                                           const int keyframe_idx,
                                                                           const std::vector<DescriptorMatch>&current_matches,
                                                                           const std::vector<SRef<Keypoint>>&current_kpoints,
                                                                           std::vector<SRef<CloudPoint>>&shared_mapPoint,
                                                                           std::vector<SRef<Point3Df>>&shared_3dpoint,
                                                                           std::vector<SRef<Point2Df>>&shared_2dpoint,
																			std::vector<DescriptorMatch> & found_matches,
                                                                           std::vector<DescriptorMatch> & remaining_matches){

                 for (int j = 0; j < current_matches.size(); ++j){
                    bool matchFound = false ;
                    for (int i = 0; i < cloud.size(); ++i) {
                        if (cloud[i]->m_visibility [keyframe_idx] == current_matches[j].getIndexInDescriptorA()) {
                            shared_mapPoint.push_back(cloud[i]) ;
                            shared_3dpoint.push_back(xpcf::utils::make_shared<Point3Df>(cloud[i]->getX(), cloud[i]->getY(),cloud[i]->getZ()));
                            shared_2dpoint.push_back(xpcf::utils::make_shared<Point2Df>(current_kpoints[current_matches[j].getIndexInDescriptorB()]->getX(),
                                                                              current_kpoints[current_matches[j].getIndexInDescriptorB()]->getY()));
							found_matches.push_back(current_matches[j]);
							matchFound = true ;
                           // break for if found?
							
                        }
                    }
                    if (!matchFound)
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
