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
#include "SolARPoseEstimationPnpOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include "opencv2/calib3d/calib3d.hpp"
#include <map>

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARPoseEstimationPnpOpencv);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

const static std::map<std::string,int> convertPnPMethod = {{"ITERATIVE", cv::SOLVEPNP_ITERATIVE},
                                                           {"P3P", cv::SOLVEPNP_P3P},
                                                           {"AP3P", cv::SOLVEPNP_AP3P},
                                                           {"EPNP", cv::SOLVEPNP_EPNP},
                                                           {"DLS", cv::SOLVEPNP_DLS},
                                                           {"UPNP", cv::SOLVEPNP_UPNP},
                                                           {"IPPE", cv::SOLVEPNP_IPPE},
                                                           {"IPPE SQUARE", cv::SOLVEPNP_IPPE_SQUARE},
                                                          };

SolARPoseEstimationPnpOpencv::SolARPoseEstimationPnpOpencv():ConfigurableBase(xpcf::toUUID<SolARPoseEstimationPnpOpencv>())
{
    declareInterface<api::solver::pose::I3DTransformFinderFrom2D3D>(this);
    declareProperty("iterationsCount", m_iterationsCount);
    declareProperty("reprojError", m_reprojError);
    declareProperty("confidence", m_confidence);
    declareProperty("minNbInliers", m_NbInliersToValidPose);
    declareProperty("method", m_method);

    m_camMatrix.create(3, 3, CV_32FC1);
    m_camDistorsion.create(5, 1, CV_32FC1);

    LOG_DEBUG(" SolARPoseEstimationOpencv constructor");
}

SolARPoseEstimationPnpOpencv::~SolARPoseEstimationPnpOpencv(){

}

FrameworkReturnCode SolARPoseEstimationPnpOpencv::estimate( const std::vector<Point2Df> & imagePoints,
                                                            const std::vector<Point3Df> & worldPoints,
                                                            Transform3Df & pose,
                                                            const Transform3Df initialPose) {

    std::vector<cv::Point2f> imageCVPoints;
    std::vector<cv::Point3f> worldCVPoints;

    int method;
    auto itr = convertPnPMethod.find(m_method);
    if (itr != convertPnPMethod.end())
        method = itr->second;
    else
        method = cv::SOLVEPNP_ITERATIVE;


    Transform3Df initialPoseInverse = initialPose.inverse();

    if (worldPoints.size()!=imagePoints.size() || worldPoints.size()< 4 ){
        LOG_WARNING("world/image points must be valid ( equal and > to 4)");
        return FrameworkReturnCode::_ERROR_  ; // vector of 2D and 3D points must have same size
    }

    for (int i=0;i<imagePoints.size();++i) {

        Point2Df point2D = imagePoints.at(i);
        Point3Df point3D = worldPoints.at(i);
        imageCVPoints.push_back(cv::Point2f(point2D.getX(), point2D.getY()));
        worldCVPoints.push_back(cv::Point3f(point3D.getX(), point3D.getY(),point3D.getZ()));
    }

    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    cv::Mat raux, taux, r33;

    
    // If initialPose is not Identity, set the useExtrinsicGuess to true. Warning, does not work on coplanar points
    if (!initialPoseInverse.isApprox(Transform3Df::Identity())){
		r33 = (cv::Mat_<float>(3, 3) << initialPoseInverse(0, 0), initialPoseInverse(0, 1), initialPoseInverse(0, 2),
										initialPoseInverse(1, 0), initialPoseInverse(1, 1), initialPoseInverse(1, 2),
										initialPoseInverse(2, 0), initialPoseInverse(2, 1), initialPoseInverse(2, 2));
		taux = (cv::Mat_<float>(3, 1) << initialPoseInverse(0, 3), initialPoseInverse(1, 3), initialPoseInverse(2, 3));
		cv::Rodrigues(r33, raux);
        
        cv::solvePnP(worldCVPoints, imageCVPoints, m_camMatrix, m_camDistorsion, raux, taux, 1, method);
    }
    else{
        cv::solvePnP(worldCVPoints, imageCVPoints, m_camMatrix, m_camDistorsion, raux, taux, 0, method);
    }
    
    raux.convertTo(Rvec, CV_32F);
    taux.convertTo(Tvec, CV_32F);

    cv::Mat_<float> rotMat(3, 3);
    cv::Rodrigues(Rvec, rotMat);

    for (int row = 0; row<3; row++){
        for (int col = 0; col<3; col++){
            pose(row,col) = rotMat(row, col);
         }
         pose(row,3) = Tvec(row);
    }
    pose(3,0)  = 0.0;
    pose(3,1)  = 0.0;
    pose(3,2)  = 0.0;
    pose(3,3)  = 1.0;

    pose = pose.inverse();

    return FrameworkReturnCode::_SUCCESS;

}

void SolARPoseEstimationPnpOpencv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {
    //TODO.. check to inverse
    this->m_camDistorsion.at<float>(0, 0)  = distorsionParams(0);
    this->m_camDistorsion.at<float>(1, 0)  = distorsionParams(1);
    this->m_camDistorsion.at<float>(2, 0)  = distorsionParams(2);
    this->m_camDistorsion.at<float>(3, 0)  = distorsionParams(3);
    this->m_camDistorsion.at<float>(4, 0)  = distorsionParams(4);

    this->m_camMatrix.at<float>(0, 0) = intrinsicParams(0,0);
    this->m_camMatrix.at<float>(0, 1) = intrinsicParams(0,1);
    this->m_camMatrix.at<float>(0, 2) = intrinsicParams(0,2);
    this->m_camMatrix.at<float>(1, 0) = intrinsicParams(1,0);
    this->m_camMatrix.at<float>(1, 1) = intrinsicParams(1,1);
    this->m_camMatrix.at<float>(1, 2) = intrinsicParams(1,2);
    this->m_camMatrix.at<float>(2, 0) = intrinsicParams(2,0);
    this->m_camMatrix.at<float>(2, 1) = intrinsicParams(2,1);
    this->m_camMatrix.at<float>(2, 2) = intrinsicParams(2,2);
}

}
}
}
