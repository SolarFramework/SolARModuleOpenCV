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
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARUndistortPointsOpencv::SolARUndistortPointsOpencv():ComponentBase(xpcf::toUUID<SolARUndistortPointsOpencv>())
{
    declareInterface<api::geom::IUndistortPoints>(this);
    //internal data for matrix
    m_camMatrix.create(3, 3, CV_32FC1);
    m_camDistortion.create(5, 1, CV_32FC1);
}

FrameworkReturnCode SolARUndistortPointsOpencv::undistort(const std::vector<SolAR::datastructure::Point2Df> & inputPoints,
                                                          const SolAR::datastructure::CameraParameters & camParams,
                                                          std::vector<SolAR::datastructure::Point2Df> & outputPoints)
{
    std::vector<cv::Point2f> ptvec; 
    std::vector<cv::Point2f> out_ptvec; 

    ptvec.resize(inputPoints.size());
    out_ptvec.resize(inputPoints.size());
    outputPoints.resize(inputPoints.size());

    for(unsigned int k = 0; k < inputPoints.size();k++){
        ptvec[k].x = inputPoints[k].getX();
        ptvec[k].y = inputPoints[k].getY();
    }

	setCameraParameters(camParams);
    
    cv::undistortPoints(ptvec,out_ptvec, m_camMatrix, m_camDistortion, cv::Mat(), m_camMatrix);

    for(unsigned int k = 0 ; k < out_ptvec.size() ; k++){
        outputPoints[k] = Point2Df( out_ptvec[k].x, out_ptvec[k].y );
    }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARUndistortPointsOpencv::undistort(const std::vector<SolAR::datastructure::Keypoint> & inputKeypoints,
                                                          const SolAR::datastructure::CameraParameters & camParams,
                                                          std::vector<SolAR::datastructure::Keypoint> & outputKeypoints)
{
	std::vector<Point2Df> inputPoints, outputPoints;
	for (const auto &it : inputKeypoints)
		inputPoints.push_back(Point2Df(it.getX(), it.getY()));
	undistort(inputPoints, camParams, outputPoints);
	outputKeypoints.resize(inputKeypoints.size());
	for (int i = 0; i < inputKeypoints.size(); ++i) {
		Keypoint kp = inputKeypoints[i];
		kp.setX(outputPoints[i].getX());
		kp.setY(outputPoints[i].getY());
		outputKeypoints[i] = kp;
	}
	return FrameworkReturnCode::_SUCCESS;
}

void SolARUndistortPointsOpencv::setCameraParameters(const SolAR::datastructure::CameraParameters & camParams)
{
    this->m_camDistortion.at<float>(0, 0) = camParams.distortion(0);
    this->m_camDistortion.at<float>(1, 0) = camParams.distortion(1);
    this->m_camDistortion.at<float>(2, 0) = camParams.distortion(2);
    this->m_camDistortion.at<float>(3, 0) = camParams.distortion(3);
    this->m_camDistortion.at<float>(4, 0) = camParams.distortion(4);

    this->m_camMatrix.at<float>(0, 0) = camParams.intrinsic(0, 0);
    this->m_camMatrix.at<float>(0, 1) = camParams.intrinsic(0, 1);
    this->m_camMatrix.at<float>(0, 2) = camParams.intrinsic(0, 2);
    this->m_camMatrix.at<float>(1, 0) = camParams.intrinsic(1, 0);
    this->m_camMatrix.at<float>(1, 1) = camParams.intrinsic(1, 1);
    this->m_camMatrix.at<float>(1, 2) = camParams.intrinsic(1, 2);
    this->m_camMatrix.at<float>(2, 0) = camParams.intrinsic(2, 0);
    this->m_camMatrix.at<float>(2, 1) = camParams.intrinsic(2, 1);
    this->m_camMatrix.at<float>(2, 2) = camParams.intrinsic(2, 2);
}

}
}
}

