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

#include "SolARUndistortPointsOpencv.h"
#include "SolAROpenCVHelper.h"

#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc.hpp>
namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARUndistortPointsOpencv)

namespace SolAR {
namespace MODULES {
namespace OPENCV {

    SolARUndistortPointsOpencv::SolARUndistortPointsOpencv():ComponentBase(xpcf::toUUID<SolARUndistortPointsOpencv>())
    {
        addInterface<api::geom::IUndistortPoints>(this);

        //internal data for matrix
        m_camMatrix.create(3, 3, CV_32FC1);
        m_camDistorsion.create(5, 1, CV_32FC1);

    }

    FrameworkReturnCode SolARUndistortPointsOpencv::undistort(const std::vector<SRef<Point2Df>> & inputPoints, std::vector<SRef<Point2Df>> & outputPoints){
        
    std::vector<cv::Point2f> ptvec; 
    std::vector<cv::Point2f> out_ptvec; 

    ptvec.resize(inputPoints.size());
    out_ptvec.resize(inputPoints.size());
    outputPoints.resize(inputPoints.size());

    for(unsigned int k = 0; k < inputPoints.size();k++){
        ptvec[k].x = inputPoints[k]->getX();
        ptvec[k].y = inputPoints[k]->getY();
    }
    
      cv::undistortPoints(ptvec,out_ptvec, m_camMatrix, m_camDistorsion);

    for(unsigned int k = 0; k < out_ptvec.size();k++){
        outputPoints[k] = ( xpcf::utils::make_shared<Point2Df>( out_ptvec[k].x, out_ptvec[k].y ));
    }
        return FrameworkReturnCode::_SUCCESS;
    }

    void SolARUndistortPointsOpencv::setIntrinsicParameters(const CamCalibration & intrinsic_parameters){
        m_intrinsic_parameters = intrinsic_parameters;

        this->m_camMatrix.at<float>(0, 0) = m_intrinsic_parameters(0, 0);
        this->m_camMatrix.at<float>(0, 1) = m_intrinsic_parameters(0, 1);
        this->m_camMatrix.at<float>(0, 2) = m_intrinsic_parameters(0, 2);
        this->m_camMatrix.at<float>(1, 0) = m_intrinsic_parameters(1, 0);
        this->m_camMatrix.at<float>(1, 1) = m_intrinsic_parameters(1, 1);
        this->m_camMatrix.at<float>(1, 2) = m_intrinsic_parameters(1, 2);
        this->m_camMatrix.at<float>(2, 0) = m_intrinsic_parameters(2, 0);
        this->m_camMatrix.at<float>(2, 1) = m_intrinsic_parameters(2, 1);
        this->m_camMatrix.at<float>(2, 2) = m_intrinsic_parameters(2, 2);
    }

    void SolARUndistortPointsOpencv::setDistorsionParameters(const CamDistortion & distorsion_parameters){
         m_distorsion_parameters = distorsion_parameters;

         this->m_camDistorsion.at<float>(0, 0) = m_distorsion_parameters(0);
         this->m_camDistorsion.at<float>(1, 0) = m_distorsion_parameters(1);
         this->m_camDistorsion.at<float>(2, 0) = m_distorsion_parameters(2);
         this->m_camDistorsion.at<float>(3, 0) = m_distorsion_parameters(3);
         this->m_camDistorsion.at<float>(4, 0) = m_distorsion_parameters(4);
    }

    


}
}
}

