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

#include "SolARPoseFinderFrom2D2DOpencv.h"
#include "SolAROpenCVHelper.h"
#include "opencv2/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "xpcf/component/ComponentFactory.h"


#include <map>

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARPoseFinderFrom2D2DOpencv);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARPoseFinderFrom2D2DOpencv::SolARPoseFinderFrom2D2DOpencv():ConfigurableBase(xpcf::toUUID<SolARPoseFinderFrom2D2DOpencv>())
{
    addInterface<api::solver::pose::I3DTransformFinderFrom2D2D>(this);
    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
    params->wrapFloat("outlierDistanceRatio", m_outlierDistanceRatio);
    params->wrapFloat("confidence", m_confidence);

    LOG_DEBUG(" SolARPoseFinderFrom2D2DOpencv constructor");
}

SolARPoseFinderFrom2D2DOpencv::~SolARPoseFinderFrom2D2DOpencv(){

}

FrameworkReturnCode SolARPoseFinderFrom2D2DOpencv::estimate(const std::vector<SRef<Point2Df>> & matchedPointsView1,
                                                            const std::vector<SRef<Point2Df>> & matchedPointsView2,
                                                            const Transform3Df& poseView1,
                                                            Transform3Df & poseView2){
    double minVal, maxVal;

    std::vector<cv::Point2f> points_view1;
    std::vector<cv::Point2f> points_view2;

    points_view1.resize(matchedPointsView1.size());
    points_view2.resize(matchedPointsView2.size());
    for( int i = 0; i < matchedPointsView1.size(); i++ ){
        points_view1[i].x=matchedPointsView1.at(i)->getX();
        points_view1[i].y=matchedPointsView1.at(i)->getY();

        points_view2[i].x=matchedPointsView2.at(i)->getX();
        points_view2[i].y=matchedPointsView2.at(i)->getY();
    }
    cv::minMaxIdx(points_view1, &minVal, &maxVal);
    cv::Point2f pp; pp.x=m_camCalibration(0,2); pp.y=m_camCalibration(1,2);
    cv::Mat Ecv = cv::findEssentialMat(points_view1, points_view2, m_camCalibration(0,0), pp, cv::RANSAC, m_confidence, m_outlierDistanceRatio * maxVal);
    cv::Mat cvRot;
    cv::Mat cvPos;

    cv::recoverPose(Ecv, points_view1, points_view2, cvRot, cvPos, m_camCalibration(0,0), pp);

    cv::Mat cvTransform;
    cvTransform = cv::Mat::eye(4, 4, CV_32F);
    cvRot.copyTo(cvTransform(cv::Rect_<float>(0,0,3,3)));
    cvPos.copyTo(cvTransform(cv::Rect_<float>(3,0,1,3)));
    Transform3Df view2Transform;
    SolAROpenCVHelper::convertCVMatToSolar(cvTransform, view2Transform);
    poseView2 = poseView1 * view2Transform;
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARPoseFinderFrom2D2DOpencv::estimate(const std::vector<SRef<Point2Df>> & matchedPointsView1,
                                                            const std::vector<SRef<Point2Df>> & matchedPointsView2,
                                                            const Transform3Df& poseView1,
                                                            Transform3Df & poseView2,
                                                            std::vector<SRef<Point2Df>>& pointsView1_inlier,
                                                            std::vector<SRef<Point2Df>>& pointsView2_inlier){
    double minVal, maxVal;

    std::vector<cv::Point2f> points_view1;
    std::vector<cv::Point2f> points_view2;

    //std::vector<uchar> status(matchedPointsView1.size());
    cv::Mat inliers;
    points_view1.resize(matchedPointsView1.size());
    points_view2.resize(matchedPointsView2.size());
    for( int i = 0; i < matchedPointsView1.size(); i++ ){
        points_view1[i].x=matchedPointsView1.at(i)->getX();
        points_view1[i].y=matchedPointsView1.at(i)->getY();

        points_view2[i].x=matchedPointsView2.at(i)->getX();
        points_view2[i].y=matchedPointsView2.at(i)->getY();
    }
    cv::minMaxIdx(points_view1, &minVal, &maxVal);
    cv::Point2f pp; pp.x=m_camCalibration(0,2); pp.y=m_camCalibration(1,2);
    cv::Mat Ecv = cv::findEssentialMat(points_view1, points_view2, m_camCalibration(0,0), pp, cv::RANSAC, m_confidence, m_outlierDistanceRatio * maxVal, inliers);
    cv::Mat cvRot;
    cv::Mat cvPos;

    cv::recoverPose(Ecv, points_view1, points_view2, cvRot, cvPos, m_camCalibration(0,0), pp, inliers);

    cv::Mat cvTransform;
    cvTransform = cv::Mat::eye(4, 4, CV_32F);
    cvRot.copyTo(cvTransform(cv::Rect_<float>(0,0,3,3)));
    cvPos.copyTo(cvTransform(cv::Rect_<float>(3,0,1,3)));
    Transform3Df view2Transform;
    SolAROpenCVHelper::convertCVMatToSolar(cvTransform, view2Transform);
    poseView2 = view2Transform.inverse() * poseView1 ;

    int nbInliers = 0;
    for (int i = 0; i < inliers.rows; i++)
    {
        if (inliers.at<bool>(i))
        {
            pointsView1_inlier.push_back(matchedPointsView1[i]);
            pointsView2_inlier.push_back(matchedPointsView2[i]);
            nbInliers++;
        }
    }
    LOG_DEBUG("Nbinliers : {} (// {})", nbInliers, inliers.rows);
    return FrameworkReturnCode::_SUCCESS;
}

void SolARPoseFinderFrom2D2DOpencv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {
    m_camCalibration = intrinsicParams;
    m_camDistorsion = distorsionParams;
}

}
}
}
