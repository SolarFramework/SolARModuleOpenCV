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

#include "SolARHomographyMatrixDecomposerOpencv.h"
#include "core/Log.h"
#include "opencv2/calib3d/calib3d.hpp"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARHomographyMatrixDecomposerOpencv)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {
SolARHomographyMatrixDecomposerOpencv::SolARHomographyMatrixDecomposerOpencv():ComponentBase(xpcf::toUUID<SolARHomographyMatrixDecomposerOpencv>())
{
    declareInterface<api::solver::pose::I2Dto3DTransformDecomposer>(this);
    LOG_DEBUG("SolARSVDFundamentalMatrixDecomposerOpencv constructor")

    m_camMatrix.create(3, 3);
    m_camDistorsion.create(5, 1);
}

bool SolARHomographyMatrixDecomposerOpencv::decompose(const Transform2Df & H, std::vector<Transform3Df> & decomposedPoses){
   //Using HZ E decomposition
    cv::Mat svd_u, svd_vt, svd_w;
    cv::Mat _H(3,3,CV_64FC1);
    for(int i  = 0; i < 3; ++i){
       for(int j = 0; j < 3; ++j){
           double e1 = H(i,j);
            _H.at<double>(i,j) =e1;// double(F(i,j));
       }
    }
    std::vector<cv::Mat>_R, _T,_N;

    cv::decomposeHomographyMat(_H,m_camMatrix,_R,_T,_N);

    Transform3Df pose_temp[16];

    for(int i =0; i <3; ++i){
      for(int j = 0; j < 3; ++j){
          pose_temp[0](i,j) = _R[0].at<double>(i,j);
          pose_temp[1](i,j) = _R[0].at<double>(i,j);
          pose_temp[2](i,j) = _R[0].at<double>(i,j);
          pose_temp[3](i,j) = _R[0].at<double>(i,j);

          pose_temp[4](i,j) = _R[1].at<double>(i,j);
          pose_temp[5](i,j) = _R[1].at<double>(i,j);
          pose_temp[6](i,j) = _R[1].at<double>(i,j);
          pose_temp[7](i,j) = _R[1].at<double>(i,j);

          pose_temp[8](i,j) = _R[2].at<double>(i,j);
          pose_temp[9](i,j) = _R[2].at<double>(i,j);
          pose_temp[10](i,j)= _R[2].at<double>(i,j);
          pose_temp[11](i,j) =_R[2].at<double>(i,j);

          pose_temp[12](i,j) = _R[3].at<double>(i,j);
          pose_temp[13](i,j) = _R[3].at<double>(i,j);
          pose_temp[14](i,j) = _R[3].at<double>(i,j);
          pose_temp[15](i,j) = _R[3].at<double>(i,j);
      }
    }
    for(int i = 0; i < 3; ++i){
      pose_temp[0](i,3) = _T[0].at<double>(i,0);
      pose_temp[1](i,3) = _T[1].at<double>(i,0);
      pose_temp[2](i,3) = _T[2].at<double>(i,0);
      pose_temp[3](i,3) = _T[3].at<double>(i,0);

      pose_temp[4](i,3) = _T[0].at<double>(i,0);
      pose_temp[5](i,3) = _T[1].at<double>(i,0);
      pose_temp[6](i,3) = _T[2].at<double>(i,0);
      pose_temp[7](i,3) = _T[3].at<double>(i,0);

      pose_temp[8](i,3) = _T[0].at<double>(i,0);
      pose_temp[9](i,3) = _T[1].at<double>(i,0);
      pose_temp[10](i,3) = _T[2].at<double>(i,0);
      pose_temp[11](i,3) = _T[3].at<double>(i,0);

      pose_temp[12](i,3) = _T[0].at<double>(i,0);
      pose_temp[13](i,3) = _T[1].at<double>(i,0);
      pose_temp[14](i,3) = _T[2].at<double>(i,0);
      pose_temp[15](i,3) = _T[3].at<double>(i,0);
    }

    for(auto & p : pose_temp){
      p(3,0) = 0.0;
      p(3,1) = 0.0;
      p(3,2) = 0.0;
      p(3,3) = 1.0;
      decomposedPoses.emplace_back(p.inverse());
    }
    return true;
}

void SolARHomographyMatrixDecomposerOpencv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {
    m_camDistorsion.at<double>(0, 0)  = (double)distorsionParams(0);
    m_camDistorsion.at<double>(1, 0)  = (double)distorsionParams(1);
    m_camDistorsion.at<double>(2, 0)  =(double) distorsionParams(2);
    m_camDistorsion.at<double>(3, 0)  = (double)distorsionParams(3);
    m_camDistorsion.at<double>(4, 0)  = (double)distorsionParams(4);

    m_camMatrix.at<double>(0, 0) = (double)intrinsicParams(0,0);
    m_camMatrix.at<double>(0, 1) = (double)intrinsicParams(0,1);
    m_camMatrix.at<double>(0, 2) = (double)intrinsicParams(0,2);
    m_camMatrix.at<double>(1, 0) = (double)intrinsicParams(1,0);
    m_camMatrix.at<double>(1, 1) = (double)intrinsicParams(1,1);
    m_camMatrix.at<double>(1, 2) = (double)intrinsicParams(1,2);
    m_camMatrix.at<double>(2, 0) = (double)intrinsicParams(2,0);
    m_camMatrix.at<double>(2, 1) = (double)intrinsicParams(2,1);
    m_camMatrix.at<double>(2, 2) = (double)intrinsicParams(2,2);
}

}
}
}
