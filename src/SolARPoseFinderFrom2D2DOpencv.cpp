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

#include "SolARPoseFinderFrom2D2DOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include "opencv2/calib3d/calib3d.hpp"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARPoseFinderFrom2D2DOpencv);

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARPoseFinderFrom2D2DOpencv::SolARPoseFinderFrom2D2DOpencv():ConfigurableBase(xpcf::toUUID<SolARPoseFinderFrom2D2DOpencv>())
{
    declareInterface<api::solver::pose::I3DTransformFinderFrom2D2D>(this);
    declareProperty("outlierDistanceRatio", m_outlierDistanceRatio);
    declareProperty("confidence", m_confidence);

    LOG_DEBUG(" SolARPoseFinderFrom2D2DOpencv constructor");
}

SolARPoseFinderFrom2D2DOpencv::~SolARPoseFinderFrom2D2DOpencv(){

}

FrameworkReturnCode SolARPoseFinderFrom2D2DOpencv::estimate(const std::vector<Point2Df> & matchedPointsView1,
                                                            const std::vector<Point2Df> & matchedPointsView2,
                                                            const Transform3Df & poseView1,
                                                            Transform3Df & poseView2,
                                                            std::vector<DescriptorMatch> & inlierMatches){
    double minVal, maxVal;

    Transform3Df poseView1Inverse = poseView1.inverse();
    LOG_INFO("Estimated pose of poseView1: \n {}", poseView1.matrix());
    LOG_INFO("Estimated pose of poseView1Inverse: \n {}", poseView1Inverse.matrix());

    std::vector<cv::Point2f> points_view1;
    std::vector<cv::Point2f> points_view2;

    cv::Mat inliers;

    if (inlierMatches.empty()) // Inliers are not defined, take all input 2D points
    {
        points_view1.resize(matchedPointsView1.size());
        points_view2.resize(matchedPointsView1.size());

        for( unsigned int i = 0; i < matchedPointsView1.size(); i++ ){
            points_view1[i].x=matchedPointsView1.at(i).getX();
            points_view1[i].y=matchedPointsView1.at(i).getY();

            points_view2[i].x=matchedPointsView2.at(i).getX();
            points_view2[i].y=matchedPointsView2.at(i).getY();
        }
    }
    else // Inliers are defined, take only them
    {
        points_view1.resize(inlierMatches.size());
        points_view2.resize(inlierMatches.size());

        for( unsigned int i = 0; i < inlierMatches.size(); i++ ){
            points_view1[i].x=matchedPointsView1.at(inlierMatches[i].getIndexInDescriptorA()).getX();
            points_view1[i].y=matchedPointsView1.at(inlierMatches[i].getIndexInDescriptorA()).getY();

            points_view2[i].x=matchedPointsView2.at(inlierMatches[i].getIndexInDescriptorB()).getX();
            points_view2[i].y=matchedPointsView2.at(inlierMatches[i].getIndexInDescriptorB()).getY();
        }
    }

    cv::minMaxIdx(points_view1, &minVal, &maxVal);
    cv::Point2f pp; pp.x=m_camCalibration(0,2); pp.y=m_camCalibration(1,2);
    cv::Mat Ecv = cv::findEssentialMat(points_view1, points_view2, m_camCalibration(0,0), pp, cv::RANSAC, m_confidence, m_outlierDistanceRatio * maxVal, inliers);
    cv::Mat cvRot;
    cv::Mat cvPos;

     std::cout <<"Essential matrix : "<<Ecv<<" "<<std::endl;

    cv::recoverPose(Ecv, points_view1, points_view2, cvRot, cvPos, m_camCalibration(0,0), pp, inliers);

    cv::Mat cvTransform;
    cvTransform = cv::Mat::eye(4, 4, CV_32F);
    cvRot.copyTo(cvTransform(cv::Rect_<float>(0,0,3,3)));
    cvPos.copyTo(cvTransform(cv::Rect_<float>(3,0,1,3)));
    Transform3Df view2Transform;
    SolAROpenCVHelper::convertCVMatToSolar(cvTransform, view2Transform);
    poseView2 = view2Transform * poseView1Inverse ;

    LOG_INFO("Estimated pose of poseView2: \n {}", poseView2.matrix());

    std::vector<DescriptorMatch> inlierMatches_output;
    if (inlierMatches.empty()) // set the inliers matches among all input 2D points return by recoverPose
    {
        for (int i = 0; i < inliers.rows; i++)
            if (inliers.at<bool>(i))
                inlierMatches_output.push_back(DescriptorMatch(i, i, 1.0f));

        LOG_DEBUG("Nbinliers : {} (// {})", inlierMatches.size(), matchedPointsView1.size());
    }
    else
    {
        for (int i = 0; i < inliers.rows; i++)
            if (inliers.at<bool>(i))
                inlierMatches_output.push_back(inlierMatches[i]);

        LOG_DEBUG("Nbinliers : {} (// {})", inlierMatches.size(), inlierMatches.size());
    }

    inlierMatches = inlierMatches_output;

    poseView2 = poseView2.inverse();
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARPoseFinderFrom2D2DOpencv::estimate(const std::vector<Keypoint> & matchedPointsView1,
                                                            const std::vector<Keypoint> & matchedPointsView2,
                                                            const Transform3Df & poseView1,
                                                            Transform3Df & poseView2,
                                                            std::vector<DescriptorMatch> & inlierMatches){
    double minVal, maxVal;

    std::vector<cv::Point2f> points_view1;
    std::vector<cv::Point2f> points_view2;
    cv::Mat inliers;

    Transform3Df poseView1Inverse = poseView1.inverse();

    if (inlierMatches.empty()) // Inliers are not defined, take all input 2D points
    {
        points_view1.resize(matchedPointsView1.size());
        points_view2.resize(matchedPointsView1.size());

        for( unsigned int i = 0; i < matchedPointsView1.size(); i++ ){
            points_view1[i].x=matchedPointsView1.at(i).getX();
            points_view1[i].y=matchedPointsView1.at(i).getY();

            points_view2[i].x=matchedPointsView2.at(i).getX();
            points_view2[i].y=matchedPointsView2.at(i).getY();
        }
    }
    else // Inliers are defined, take only them
    {
        points_view1.resize(inlierMatches.size());
        points_view2.resize(inlierMatches.size());

        for( unsigned int i = 0; i < inlierMatches.size(); i++ ){
            points_view1[i].x=matchedPointsView1.at(inlierMatches[i].getIndexInDescriptorA()).getX();
            points_view1[i].y=matchedPointsView1.at(inlierMatches[i].getIndexInDescriptorA()).getY();

            points_view2[i].x=matchedPointsView2.at(inlierMatches[i].getIndexInDescriptorB()).getX();
            points_view2[i].y=matchedPointsView2.at(inlierMatches[i].getIndexInDescriptorB()).getY();
        }
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
    poseView2 = view2Transform * poseView1Inverse ;

    std::vector<DescriptorMatch> inlierMatches_output;
    if (inlierMatches.empty()) // set the inliers matches among all input 2D points return by recoverPose
    {
        for (int i = 0; i < inliers.rows; i++)
            if (inliers.at<bool>(i))
                inlierMatches_output.push_back(DescriptorMatch(i, i, 1.0f));

        LOG_DEBUG("Nbinliers : {} (// {})", inlierMatches.size(), matchedPointsView1.size());
    }
    else
    {
        for (int i = 0; i < inliers.rows; i++)
            if (inliers.at<bool>(i))
                inlierMatches_output.push_back(inlierMatches[i]);

        LOG_DEBUG("Nbinliers : {} (// {})", inlierMatches.size(), inlierMatches.size());
    }

    poseView2 = poseView2.inverse();
    inlierMatches = inlierMatches_output;
    return FrameworkReturnCode::_SUCCESS;
}

void SolARPoseFinderFrom2D2DOpencv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {
    m_camCalibration = intrinsicParams;
    m_camDistorsion = distorsionParams;
}

}
}
}
