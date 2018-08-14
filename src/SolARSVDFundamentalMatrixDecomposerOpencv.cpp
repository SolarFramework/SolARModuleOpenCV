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
#include <iostream>

#include "SolARSVDFundamentalMatrixDecomposerOpencv.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include "xpcf/component/ComponentFactory.h"

#include <map>

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARSVDFundamentalMatrixDecomposerOpencv);

namespace xpcf  = org::bcom::xpcf;
namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARSVDFundamentalMatrixDecomposerOpencv::SolARSVDFundamentalMatrixDecomposerOpencv():ComponentBase(xpcf::toUUID<SolARSVDFundamentalMatrixDecomposerOpencv>()){
    addInterface<api::solver::pose::I2DTO3DTransformDecomposer>(this);
    LOG_DEBUG("SolARSVDFundamentalMatrixDecomposerOpencv constructor")

    m_camMatrix.create(3, 3);
    m_camDistorsion.create(5, 1);
}

SolARSVDFundamentalMatrixDecomposerOpencv::~SolARSVDFundamentalMatrixDecomposerOpencv(){
}

void SolARSVDFundamentalMatrixDecomposerOpencv::takeSVDOfE(cv::Mat_<double>& E,
                                                           cv::Mat& svd_u,
                                                           cv::Mat& svd_vt,
                                                           cv::Mat& svd_w) {
    //Using OpenCV's SVD
    cv::SVD svd(E, cv::SVD::MODIFY_A);
    svd_u = svd.u;
    svd_vt = svd.vt;
    svd_w = svd.w;
}

void SolARSVDFundamentalMatrixDecomposerOpencv::fillposes(const cv::Mat_<double>& R1,
                                                          const cv::Mat_<double>& R2,
                                                          const cv::Mat_<double>& t1,
                                                          const cv::Mat_<double>& t2,
                                                          std::vector<Transform3Df>&decomposedPoses){
        decomposedPoses.resize(4);

        decomposedPoses[0](0,0) = R1(0,0);decomposedPoses[0](0,1) = R1(0,1);decomposedPoses[0](0,2) = R1(0,2);decomposedPoses[0](0,3) = t1(0);
        decomposedPoses[0](1,0) = R1(1,0);decomposedPoses[0](1,1) = R1(1,1);decomposedPoses[0](1,2) = R1(1,2);decomposedPoses[0](1,3) = t1(1);
        decomposedPoses[0](2,0) = R1(2,0);decomposedPoses[0](2,1) = R1(2,1);decomposedPoses[0](2,2) = R1(2,2);decomposedPoses[0](2,3) = t1(2);
        decomposedPoses[0](3,0) = 0.0    ;decomposedPoses[0](3,1) = 0.0    ;decomposedPoses[0](3,2) = 0.0    ;decomposedPoses[0](3,3) = 1.0;

        decomposedPoses[1](0,0) = R1(0,0);decomposedPoses[1](0,1) = R1(0,1);decomposedPoses[1](0,2) = R1(0,2);decomposedPoses[1](0,3) = t2(0);
        decomposedPoses[1](1,0) = R1(1,0);decomposedPoses[1](1,1) = R1(1,1);decomposedPoses[1](1,2) = R1(1,2);decomposedPoses[1](1,3) = t2(1);
        decomposedPoses[1](2,0) = R1(2,0);decomposedPoses[1](2,1) = R1(2,1);decomposedPoses[1](2,2) = R1(2,2);decomposedPoses[1](2,3) = t2(2);
        decomposedPoses[1](3,0) = 0.0    ;decomposedPoses[1](3,1) = 0.0    ;decomposedPoses[1](3,2) = 0.0    ;decomposedPoses[1](3,3) = 1.0;

        decomposedPoses[2](0,0) = R2(0,0);decomposedPoses[2](0,1) = R2(0,1);decomposedPoses[2](0,2) = R2(0,2);decomposedPoses[2](0,3) = t1(0);
        decomposedPoses[2](1,0) = R2(1,0);decomposedPoses[2](1,1) = R2(1,1);decomposedPoses[2](1,2) = R2(1,2);decomposedPoses[2](1,3) = t1(1);
        decomposedPoses[2](2,0) = R2(2,0);decomposedPoses[2](2,1) = R2(2,1);decomposedPoses[2](2,2) = R2(2,2);decomposedPoses[2](2,3) = t1(2);
        decomposedPoses[2](3,0) = 0.0    ;decomposedPoses[2](3,1) = 0.0    ;decomposedPoses[2](3,2) = 0.0    ;decomposedPoses[2](3,3) = 1.0;

        decomposedPoses[3](0,0) = R2(0,0);decomposedPoses[3](0,1) = R2(0,1);decomposedPoses[3](0,2) = R2(0,2);decomposedPoses[3](0,3) = t2(0);
        decomposedPoses[3](1,0) = R2(1,0);decomposedPoses[3](1,1) = R2(1,1);decomposedPoses[3](1,2) = R2(1,2);decomposedPoses[3](1,3) = t2(1);
        decomposedPoses[3](2,0) = R2(2,0);decomposedPoses[3](2,1) = R2(2,1);decomposedPoses[3](2,2) = R2(2,2);decomposedPoses[3](2,3) = t2(2);
        decomposedPoses[3](3,0) = 0.0    ;decomposedPoses[3](3,1) = 0.0    ;decomposedPoses[3](3,2) = 0.0    ;decomposedPoses[3](3,3) = 1.0;

}

bool SolARSVDFundamentalMatrixDecomposerOpencv::decomposeInternal(cv::Mat_<double>& E,
                                                                  cv::Mat_<double>& R1,
                                                                  cv::Mat_<double>& R2,
                                                                  cv::Mat_<double>& t1,
                                                                  cv::Mat_<double>& t2){


        cv::Mat svd_u, svd_vt, svd_w;
       takeSVDOfE(E, svd_u, svd_vt, svd_w);

        //check if first and second singular values are the same (as they should be)
        double singular_values_ratio = fabsf(svd_w.at<double>(0) / svd_w.at<double>(1));
        if (singular_values_ratio>1.0) singular_values_ratio = 1.0 / singular_values_ratio; // flip ratio to keep it [0,1]
        if (singular_values_ratio < 0.7) {
            LOG_WARNING("singular values are too far apart");
            return false;
        }

        cv::Matx33d W(0, -1, 0,
                      1, 0, 0,
                      0, 0, 1);
        cv::Matx33d Wt(0, 1, 0,
                      -1, 0, 0,
                       0, 0, 1);
        std::cout<<"svd_u :"<<std::endl<<svd_u<<std::endl;
        std::cout<<"svd_w :"<<std::endl<<svd_w<<std::endl;

        R1 = svd_u * cv::Mat(W) * svd_vt; //H
        R2 = svd_u * cv::Mat(Wt) * svd_vt; //
        t1 = svd_u.col(2); //u3
        t2 = -svd_u.col(2); //-u3
        return true;
}

bool SolARSVDFundamentalMatrixDecomposerOpencv::decompose(const Transform2Df&F, std::vector<Transform3Df>& decomposedPoses){
    //Using HZ E decomposition
       cv::Mat svd_u, svd_vt, svd_w;
       cv::Mat _F(3,3,CV_64FC1);
       for(int i  = 0; i < 3; ++i){
           for(int j = 0; j < 3; ++j){
               double e1 = F(i,j);
               _F.at<double>(i,j) =e1;// double(F(i,j));                       }
            }
       }

      cv::Mat_<double> E = m_camMatrix.t() * _F * m_camMatrix; //
      cv::Mat_<double> R1(3, 3);
      cv::Mat_<double> R2(3, 3);
      cv::Mat_<double> t(1, 3);
      //cv::Mat_<double> t1(1, 3);
      //cv::Mat_<double> t2(1, 3);

      cv::decomposeEssentialMat(E, R1, R2, t);
/*
      if(!decomposeInternal(E,R1,R2,t1,t2)){
          return false;
      }
      if(cv::determinant(R1)+ 1.0 < 1e-09){
          E = -E;
          decomposeInternal(E,R1,R2,t1,t2);
          fillposes(R1,R2, t1,t2, decomposedPoses);
          return true;
      }
*/
      //fillposes(R1,R2,t1,t2,decomposedPoses);
      fillposes(R1,R2,t,-t,decomposedPoses);
      return true;
}

void SolARSVDFundamentalMatrixDecomposerOpencv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {
    //TODO.. check to inverse
    this->m_camDistorsion.at<double>(0, 0)  = (double)distorsionParams(0);
    this->m_camDistorsion.at<double>(1, 0)  = (double)distorsionParams(1);
    this->m_camDistorsion.at<double>(2, 0)  =(double) distorsionParams(2);
    this->m_camDistorsion.at<double>(3, 0)  = (double)distorsionParams(3);
    this->m_camDistorsion.at<double>(4, 0)  = (double)distorsionParams(4);

    this->m_camMatrix.at<double>(0, 0) = (double)intrinsicParams(0,0);
    this->m_camMatrix.at<double>(0, 1) = (double)intrinsicParams(0,1);
    this->m_camMatrix.at<double>(0, 2) = (double)intrinsicParams(0,2);
    this->m_camMatrix.at<double>(1, 0) = (double)intrinsicParams(1,0);
    this->m_camMatrix.at<double>(1, 1) = (double)intrinsicParams(1,1);
    this->m_camMatrix.at<double>(1, 2) = (double)intrinsicParams(1,2);
    this->m_camMatrix.at<double>(2, 0) = (double)intrinsicParams(2,0);
    this->m_camMatrix.at<double>(2, 1) = (double)intrinsicParams(2,1);
    this->m_camMatrix.at<double>(2, 2) = (double)intrinsicParams(2,2);
}


}
}
}
