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

#include "SolARSVDFundamentalMatrixDecomposerOpencv.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include "ComponentFactory.h"

#include <map>

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARSVDFundamentalMatrixDecomposerOpencv);

namespace SolAR {
using namespace datastructure;
    namespace MODULES {
        namespace OPENCV {
            SolARSVDFundamentalMatrixDecomposerOpencv::SolARSVDFundamentalMatrixDecomposerOpencv(){
                setUUID(SolARSVDFundamentalMatrixDecomposerOpencv::UUID);
                addInterface<api::solver::pose::IFundamentalMatrixDecomposer>(this,api::solver::pose::IFundamentalMatrixDecomposer::UUID, "interface api::solver::pose::IFundamentalMatrixDecomposer");
                LOG_DEBUG("SolARSVDFundamentalMatrixDecomposerOpencv constructor")
            }

            void SolARSVDFundamentalMatrixDecomposerOpencv::takeSVDOfE(cv::Mat_<double>& E,cv::Mat& svd_u, cv::Mat& svd_vt,cv::Mat& svd_w) {
                //Using OpenCV's SVD
                cv::SVD svd(E, cv::SVD::MODIFY_A);
                svd_u = svd.u;
                svd_vt = svd.vt;
                svd_w = svd.w;
            }

            bool SolARSVDFundamentalMatrixDecomposerOpencv::decompose(const Transform2Df&F,const CamCalibration&K, const CamDistortion& dist, std::vector<SRef<Pose>>& decomposedPoses){
               //Using HZ E decomposition
                   cv::Mat svd_u, svd_vt, svd_w;
                   cv::Mat _K(3,3,CV_64FC1);
                   cv::Mat _F(3,3,CV_64FC1);
                   for(int i  = 0; i < 3; ++i){
                       for(int j = 0; j < 3; ++j){
                           double e0 = K(i,j);
                           double e1 = F(i,j);
                            _K.at<double>(i,j) =e0;//double(K(i,j));
                            _F.at<double>(i,j) =e1;// double(F(i,j));
                       }
                   }
                  std::cout<<"inside decompose: "<<std::endl;
                  std::cout<<"Kcv: "<<std::endl;
                  std::cout<<_K<<std::endl;
                  std::cout<<"Fcv: "<<std::endl;
                  std::cout<<_F<<std::endl;
                  cv::Mat_<double> E = _K.t() * _F * _K; //
                  std::cout<<"Ecv: "<<std::endl;
                  std::cout<<E<<std::endl;

                  takeSVDOfE(E, svd_u, svd_vt, svd_w);
                  std::cout<<"take svd done.."<<std::endl;
                   //check if first and second singular values are the same (as they should be)
                   double singular_values_ratio = fabsf(svd_w.at<double>(0) / svd_w.at<double>(1));
                   if (singular_values_ratio>1.0) singular_values_ratio = 1.0 / singular_values_ratio; // flip ratio to keep it [0,1]
                   if (singular_values_ratio < 0.7) {
                       std::cout << "singular values are too far apart\n";
                       return false;
                   }

                   cv::Matx33d W(0, -1, 0,
                                 1, 0, 0,
                                 0, 0, 1);
                   cv::Matx33d Wt(0, 1, 0,
                                 -1, 0, 0,
                                  0, 0, 1);

                   cv::Mat_<double> R1(3, 3);
                   cv::Mat_<double> R2(3, 3);

                   cv::Mat_<double> t1(1, 3);
                   cv::Mat_<double> t2(1, 3);

                   R1 = svd_u * cv::Mat(W) * svd_vt; //HZ 9.19
                   R2 = svd_u * cv::Mat(Wt) * svd_vt; //HZ 9.19
                   t1 = svd_u.col(2); //u3
                   t2 = -svd_u.col(2); //u3


                   sptrnms::shared_ptr<Pose> pose_temp[4];
                   for(int p = 0; p < 4; ++p)
                       pose_temp[p] = sptrnms::make_shared<Pose>();

                   for(int i =0; i <3; ++i){
                       for(int j = 0; j < 3; ++j){
                           pose_temp[0]->m_poseTransform(i,j) = R1(i,j);
                           pose_temp[1]->m_poseTransform(i,j) = R1(i,j);
                           pose_temp[2]->m_poseTransform(i,j) = R2(i,j);
                           pose_temp[3]->m_poseTransform(i,j) = R2(i,j);

                       }
                   }
                   for(int i = 0; i < 3; ++i){
                       pose_temp[0]->m_poseTransform(i,3) = t1(i);
                       pose_temp[1]->m_poseTransform(i,3) = t2(i);
                       pose_temp[2]->m_poseTransform(i,3) = t1(i);
                       pose_temp[3]->m_poseTransform(i,3) = t2(i);
                   }

                   for(int p = 0; p < 4; ++p){
                       pose_temp[p]->m_poseTransform(3,0) = 0.0;
                       pose_temp[p]->m_poseTransform(3,1) = 0.0;
                       pose_temp[p]->m_poseTransform(3,2) = 0.0;
                       pose_temp[p]->m_poseTransform(3,3) = 1.0;
                       decomposedPoses.push_back(pose_temp[p]);
                   }



                //   decomposedPoses.push_back(p1);decomposedPoses.push_back(p2);
                //   decomposedPoses.push_back(p3);decomposedPoses.push_back(p4);

				   return true;
            }
        }
    }
}
