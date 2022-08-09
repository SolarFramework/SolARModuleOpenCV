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

#include "SolARProjectOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include "opencv2/calib3d/calib3d.hpp"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARProjectOpencv);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARProjectOpencv::SolARProjectOpencv():ConfigurableBase(xpcf::toUUID<SolARProjectOpencv>())
{
    declareInterface<api::geom::IProject>(this);

    m_camMatrix.create(3, 3, CV_32FC1);
    m_camDistorsion.create(5, 1, CV_32FC1);

    LOG_DEBUG(" SolARProjectOpencv constructor");
}

SolARProjectOpencv::~SolARProjectOpencv(){

}

FrameworkReturnCode projectCV(const std::vector<cv::Point3f> & inputPoints, std::vector<Point2Df> & imagePoints, const Transform3Df& pose, const cv::Mat & intrinsicParams, const cv::Mat & distorsionParams )
{
    std::vector<cv::Point2f> cvImagePoints;

    Transform3Df poseInv = pose.inverse();

    cv::Mat rotMat, rvec;
    rotMat = (cv::Mat_<float>(3, 3) << poseInv(0, 0), poseInv(0, 1), poseInv(0, 2),
                                     poseInv(1, 0), poseInv(1, 1), poseInv(1, 2),
                                     poseInv(2, 0), poseInv(2, 1), poseInv(2, 2));

    cv::Mat tvec(3, 1, SolAROpenCVHelper::inferOpenCVType<float>());
    tvec.at<float>(0, 0) = poseInv(0, 3);
    tvec.at<float>(1, 0) = poseInv(1, 3);
    tvec.at<float>(2, 0) = poseInv(2, 3);
    cv::Rodrigues(rotMat, rvec);

    cv::projectPoints(inputPoints, rvec, tvec, intrinsicParams, distorsionParams, cvImagePoints);

    imagePoints.clear();
    for (auto cvPoint : cvImagePoints)
        imagePoints.push_back(Point2Df((float)cvPoint.x, (float)cvPoint.y));

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARProjectOpencv::project(const std::vector<SolAR::datastructure::Point3Df> & inputPoints,
												const SolAR::datastructure::Transform3Df& pose,
												const SolAR::datastructure::CameraParameters & camParams,
												std::vector<SolAR::datastructure::Point2Df> & imagePoints)
{
    std::vector<cv::Point3f> cvWorldPoints;
    for (auto point : inputPoints)
        cvWorldPoints.push_back(cv::Point3f(point.getX(), point.getY(), point.getZ()));

	setCameraParameters(camParams);
	return projectCV(cvWorldPoints, imagePoints, pose, m_camMatrix, m_camDistorsion);
}

FrameworkReturnCode SolARProjectOpencv::project(const std::vector<SRef<SolAR::datastructure::CloudPoint>> & inputPoints,
												const SolAR::datastructure::Transform3Df& pose,
												const SolAR::datastructure::CameraParameters & camParams,
												std::vector<SolAR::datastructure::Point2Df> & imagePoints)
{
    std::vector<cv::Point3f> cvWorldPoints;
    for (auto point : inputPoints)
        cvWorldPoints.push_back(cv::Point3f(point->getX(), point->getY(), point->getZ()));

	setCameraParameters(camParams);
    return projectCV(cvWorldPoints, imagePoints, pose, m_camMatrix, m_camDistorsion);
}

void SolARProjectOpencv::setCameraParameters(const SolAR::datastructure::CameraParameters & camParams) 
{
	this->m_camDistorsion.at<float>(0, 0) = camParams.distortion(0);
	this->m_camDistorsion.at<float>(1, 0) = camParams.distortion(1);
	this->m_camDistorsion.at<float>(2, 0) = camParams.distortion(2);
	this->m_camDistorsion.at<float>(3, 0) = camParams.distortion(3);
	this->m_camDistorsion.at<float>(4, 0) = camParams.distortion(4);

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
