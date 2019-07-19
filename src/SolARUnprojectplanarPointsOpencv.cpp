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

#include "SolARUnprojectPlanarPointsOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARUnprojectPlanarPointsOpencv);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARUnprojectPlanarPointsOpencv::SolARUnprojectPlanarPointsOpencv():ConfigurableBase(xpcf::toUUID<SolARUnprojectPlanarPointsOpencv>())
{
    declareInterface<api::geom::IUnproject>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();

    m_camMatrix.create(3, 3, CV_32FC1);
    m_camDistorsion.create(5, 1, CV_32FC1);

    LOG_DEBUG(" SolARUnprojectPlanarPositionOpencv constructor");
}

SolARUnprojectPlanarPointsOpencv::~SolARUnprojectPlanarPointsOpencv(){

}

FrameworkReturnCode unprojectOCV(const std::vector<cv::Point2f>& imagePoints, std::vector<Point3Df>& worldPoints, const Transform3Df& pose, const cv::Mat &m_camMatrix, const cv::Mat &m_camDistorsion)
{	
	// undistort 2D points	
	std::vector<cv::Point2f> correctedImagePoints;
	cv::undistortPoints(imagePoints, correctedImagePoints, m_camMatrix, m_camDistorsion);

	// get unproject transformation matrix
	Transform3Df poseInv = pose.inverse();
	cv::Mat ext = (cv::Mat_<float>(3, 3) <<
		poseInv(0, 0), poseInv(0, 1), poseInv(0, 3),
		poseInv(1, 0), poseInv(1, 1), poseInv(1, 3),
		poseInv(2, 0), poseInv(2, 1), poseInv(2, 3));	
	cv::Mat unProj = ext.inv();

	// unproject points
	for (auto it : correctedImagePoints) {
		cv::Mat pt2D = (cv::Mat_<float>(3, 1) << it.x, it.y, 1.f);
		cv::Mat pt3D = unProj * pt2D;
        worldPoints.push_back(Point3Df(pt3D.at<float>(0, 0) / pt3D.at<float>(2, 0), pt3D.at<float>(1, 0) / pt3D.at<float>(2, 0), 0.f));
	}

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARUnprojectPlanarPointsOpencv::unproject(const std::vector<Point2Df> & imagePoints,
                                                                std::vector<Point3Df> & worldPoints,
                                                                const Transform3Df& pose)
{
    if (imagePoints.empty())
        return FrameworkReturnCode::_ERROR_;
    std::vector<cv::Point2f> cvPoints;
    for (auto point : imagePoints)
        cvPoints.push_back(cv::Point2f(point.getX(), point.getY()));

    return unprojectOCV(cvPoints, worldPoints, pose, m_camMatrix, m_camDistorsion);
}

FrameworkReturnCode SolARUnprojectPlanarPointsOpencv::unproject(const std::vector<Keypoint> & imageKeypoints,
                                                                std::vector<Point3Df> & worldPoints,
                                                                const Transform3Df& pose)
{
    if (imageKeypoints.empty())
        return FrameworkReturnCode::_ERROR_;
    std::vector<cv::Point2f> cvPoints;
    for (auto point : imageKeypoints)
        cvPoints.push_back(cv::Point2f(point.getX(), point.getY()));

    return unprojectOCV(cvPoints, worldPoints, pose, m_camMatrix, m_camDistorsion);
}

void SolARUnprojectPlanarPointsOpencv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {

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
