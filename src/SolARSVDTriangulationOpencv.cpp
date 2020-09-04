

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

#include "SolARSVDTriangulationOpencv.h"
#include "SolAROpenCVHelper.h"
#include "core/Log.h"
#include "opencv2/calib3d/calib3d.hpp"

namespace xpcf  = org::bcom::xpcf;

#define EPSILON 0.0001
#define intrpmnmx(val,min,max) (max==min ? 0.0 : ((val)-min)/(max-min))

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARSVDTriangulationOpencv);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

SolARSVDTriangulationOpencv::SolARSVDTriangulationOpencv():ComponentBase(xpcf::toUUID<SolARSVDTriangulationOpencv>())
{
    declareInterface<api::solver::map::ITriangulator>(this);
    LOG_DEBUG(" SolARSVDTriangulationOpencv constructor");
    m_camMatrix.create(3, 3);
    m_camDistorsion.create(5, 1);
}

SolARSVDTriangulationOpencv::~SolARSVDTriangulationOpencv(){

}


cv::Mat_<double> SolARSVDTriangulationOpencv::iterativeLinearTriangulation(const cv::Point3d & u1,
                                                                           const cv::Mat & P1,
                                                                           const cv::Point3d & u2,
                                                                           const cv::Mat & P2){

	double w1 = 1, w2 = 1;
	cv::Mat_<double> X;
	for (int i = 0; i < 10; i++) {
		cv::Mat A(4, 4, CV_64F);
		A.row(0) = (u1.x * P1.row(2) - P1.row(0)) / w1;
		A.row(1) = (u1.y * P1.row(2) - P1.row(1)) / w1;
		A.row(2) = (u2.x * P2.row(2) - P2.row(0)) / w2;
		A.row(3) = (u2.y * P2.row(2) - P2.row(1)) / w2;
		cv::SVD::solveZ(A, X);

		double new_w1 = cv::Mat(P1.row(2) * X).at<double>(0);
		double new_w2 = cv::Mat(P2.row(2) * X).at<double>(0);

		if (std::abs(w1 - new_w1) <= EPSILON && std::abs(w2 - new_w2) <= EPSILON)
		{
			break;
		}

		w1 = new_w1;
		w2 = new_w2;
	}
	if (X.at<double>(3) != 0)
		X = X / X.at<double>(3);
    return X;
}

cv::Mat_<double> SolARSVDTriangulationOpencv::linearTriangulation(const cv::Point3d & u1,
                                                                  const cv::Mat & P1,
                                                                  const cv::Point3d & u2,
                                                                  const cv::Mat & P2){

	cv::Mat A(4, 4, CV_64F);
	A.row(0) = u1.x * P1.row(2) - P1.row(0);
	A.row(1) = u1.y * P1.row(2) - P1.row(1);
	A.row(2) = u2.x * P2.row(2) - P2.row(0);
	A.row(3) = u2.y * P2.row(2) - P2.row(1);
	cv::Mat w, u, vt;
	cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
	
	cv::Mat_<double> X = vt.row(3).t();

	if (X.at<double>(3) != 0)
		X = X / X.at<double>(3);

    return X;
}

double SolARSVDTriangulationOpencv::getReprojectionErrorCloud(const std::vector<SRef<CloudPoint>> & original){
    double err = 0.f;
    for(auto const & cloudpoint : original){
        err += cloudpoint->getReprojError();
    }
    return (err/=double(original.size()));
}


double SolARSVDTriangulationOpencv::triangulate(const std::vector<Point2Df> & pointsView1,
                                                const std::vector<Point2Df> & pointsView2,
                                                const std::vector<DescriptorMatch> & matches,
                                                const std::pair<unsigned int,unsigned int> & working_views,
                                                const Transform3Df & poseView1,
                                                const Transform3Df & poseView2,
                                                std::vector<SRef<CloudPoint>> & pcloud){
	pcloud.clear();

    Transform3Df poseView1Inverse = poseView1.inverse();
    Transform3Df poseView2Inverse = poseView2.inverse();
    
	cv::Mat Pose1 = (cv::Mat_<double>(3, 4) << poseView1Inverse(0, 0), poseView1Inverse(0, 1), poseView1Inverse(0, 2), poseView1Inverse(0, 3),
		poseView1Inverse(1, 0), poseView1Inverse(1, 1), poseView1Inverse(1, 2), poseView1Inverse(1, 3),
		poseView1Inverse(2, 0), poseView1Inverse(2, 1), poseView1Inverse(2, 2), poseView1Inverse(2, 3));



	cv::Mat Pose2 = (cv::Mat_<double>(3, 4) << poseView2Inverse(0, 0), poseView2Inverse(0, 1), poseView2Inverse(0, 2), poseView2Inverse(0, 3),
		poseView2Inverse(1, 0), poseView2Inverse(1, 1), poseView2Inverse(1, 2), poseView2Inverse(1, 3),
		poseView2Inverse(2, 0), poseView2Inverse(2, 1), poseView2Inverse(2, 2), poseView2Inverse(2, 3));

    // Create a vector to store the reprojection error of each triangulated 3D points
    std::vector<double> reproj_error;
    // unsigned int pts_size = pt2d_1.size();
    unsigned int pts_size = matches.size();

    // KPose 1 and KPose2 represent the transformations from 3D space to 2D image (K*[R|T]).
    cv::Mat_<double> KPose1;
    KPose1 = m_camMatrix * Pose1;
	Vector3f meanCamCenter((poseView1(0, 3) + poseView2(0, 3)) / 2, (poseView1(1, 3) + poseView2(1, 3)) / 2, (poseView1(2, 3) + poseView2(2, 3)) / 2);

    for (int i = 0; i<pts_size; i++) {
        cv::Point2f kp1 = cv::Point2f(pointsView1[matches[i].getIndexInDescriptorA()].getX(),pointsView1[matches[i].getIndexInDescriptorA()].getY());
        cv::Point3d u1(kp1.x, kp1.y, 1.0);
        // um1 represents an homogenous point in 3D camera space positionned on the image plan
        cv::Mat_<double> um1 = m_Kinv * cv::Mat_<double>(u1);
        u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

        cv::Point2f kp2 = cv::Point2f(pointsView2[matches[i].getIndexInDescriptorB()].getX(),pointsView2[matches[i].getIndexInDescriptorB()].getY());
        cv::Point3d u2(kp2.x, kp2.y, 1.0);
        // um1 represents an homogenous point in 3D camera space positionned on the image plan
        cv::Mat_<double> um2 = m_Kinv * cv::Mat_<double>(u2);
        u2.x = um2(0); u2.y = um2(1); u2.z = um2(2);

        // Compute the position of the 3D point projected in u1 for the camera1 with Pose1 and projected in u2 for the camera 2 with Pose 2
        cv::Mat_<double> X = iterativeLinearTriangulation(u1, Pose1, u2, Pose2);

        // Reproject this point on the image plane of the second camera
        cv::Mat_<double> xPt_img1 = KPose1 * X;				//reproject
        cv::Point2f xPt_img_1(xPt_img1(0) / xPt_img1(2), xPt_img1(1) / xPt_img1(2));

        double reprj_err = norm(xPt_img_1 - kp1);
        reproj_error.push_back(reprj_err);

        std::map<unsigned int, unsigned int> visibility;

        visibility[working_views.first]  = matches[i].getIndexInDescriptorA();
        visibility[working_views.second] = matches[i].getIndexInDescriptorB();
		SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(X(0), X(1), X(2), 0.0, 0.0, 0.0, meanCamCenter(0) - X(0), meanCamCenter(1) - X(1), meanCamCenter(2) - X(2), reprj_err, visibility);
        pcloud.push_back(cp);
    }
    cv::Scalar mse = cv::mean(reproj_error);
    return mse[0];
}

double SolARSVDTriangulationOpencv::triangulate(const std::vector<Keypoint> & keypointsView1,
                                                const std::vector<Keypoint> & keypointsView2,
                                                const std::vector<DescriptorMatch> & matches,
                                                const std::pair<unsigned int,unsigned int> & working_views,
                                                const Transform3Df & poseView1,
                                                const Transform3Df & poseView2,
                                                std::vector<SRef<CloudPoint>> & pcloud){
	pcloud.clear();

    Transform3Df poseView1Inverse = poseView1.inverse();
    Transform3Df poseView2Inverse = poseView2.inverse();
	cv::Mat Pose1 = (cv::Mat_<double>(3, 4) << poseView1Inverse(0, 0), poseView1Inverse(0, 1), poseView1Inverse(0, 2), poseView1Inverse(0, 3),
                       poseView1Inverse(1, 0),poseView1Inverse(1, 1),poseView1Inverse(1, 2), poseView1Inverse(1, 3),
                       poseView1Inverse(2, 0),poseView1Inverse(2, 1),poseView1Inverse(2, 2), poseView1Inverse(2, 3));



	cv::Mat Pose2 = (cv::Mat_<double>(3, 4) << poseView2Inverse(0, 0), poseView2Inverse(0, 1), poseView2Inverse(0, 2), poseView2Inverse(0, 3),
                      poseView2Inverse(1, 0),poseView2Inverse(1, 1),poseView2Inverse(1, 2), poseView2Inverse(1, 3),
                      poseView2Inverse(2, 0),poseView2Inverse(2, 1),poseView2Inverse(2, 2), poseView2Inverse(2, 3));

    double t = cv::getTickCount();
    // Create a vector to store the reprojection error of each triangulated 3D points
    std::vector<double> reproj_error;
    // unsigned int pts_size = pt2d_1.size();
    unsigned int pts_size = matches.size();

    // KPose 1 and KPose2 represent the transformations from 3D space to 2D image (K*[R|T]).
    cv::Mat_<double> KPose1;
    KPose1 = m_camMatrix * Pose1;
	Vector3f meanCamCenter((poseView1(0, 3) + poseView2(0, 3)) / 2, (poseView1(1, 3) + poseView2(1, 3)) / 2, (poseView1(2, 3) + poseView2(2, 3)) / 2);

    for (int i = 0; i<pts_size; i++) {
        cv::Point2f kp1 = cv::Point2f(keypointsView1[matches[i].getIndexInDescriptorA()].getX(), keypointsView1[matches[i].getIndexInDescriptorA()].getY());
        cv::Point3d u1(kp1.x, kp1.y, 1.0);
        // um1 represents an homogenous point in 3D camera space positionned on the image plan
        cv::Mat_<double> um1 = m_Kinv * cv::Mat_<double>(u1);
        u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

        cv::Point2f kp2 = cv::Point2f(keypointsView2[matches[i].getIndexInDescriptorB()].getX(), keypointsView2[matches[i].getIndexInDescriptorB()].getY());
        cv::Point3d u2(kp2.x, kp2.y, 1.0);
        // um1 represents an homogenous point in 3D camera space positionned on the image plan
        cv::Mat_<double> um2 = m_Kinv * cv::Mat_<double>(u2);
        u2.x = um2(0); u2.y = um2(1); u2.z = um2(2);

        // Compute the position of the 3D point projected in u1 for the camera1 with Pose1 and projected in u2 for the camera 2 with Pose 2
        cv::Mat_<double> X = iterativeLinearTriangulation(u1, Pose1, u2, Pose2);

        // Reproject this point on the image plane of the second camera
        cv::Mat_<double> xPt_img1 = KPose1 * X;				//reproject
        cv::Point2f xPt_img_1(xPt_img1(0) / xPt_img1(2), xPt_img1(1) / xPt_img1(2));

        double reprj_err = norm(xPt_img_1 - kp1);

        reproj_error.push_back(reprj_err);

        std::map<unsigned int, unsigned int> visibility;

        visibility[working_views.first]  = matches[i].getIndexInDescriptorA();
        visibility[working_views.second] = matches[i].getIndexInDescriptorB();		

		SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(X(0), X(1), X(2), 0.0, 0.0, 0.0, meanCamCenter(0) - X(0), meanCamCenter(1) - X(1), meanCamCenter(2) - X(2), reprj_err, visibility);
        pcloud.push_back(cp);
    }
    cv::Scalar mse = cv::mean(reproj_error);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    return mse[0];
}

double SolARSVDTriangulationOpencv::triangulate(const std::vector<Keypoint>& keypointsView1, 
												const std::vector<Keypoint>& keypointsView2, 
												const SRef<DescriptorBuffer>& descriptor1,
												const SRef<DescriptorBuffer>& descriptor2,
												const std::vector<DescriptorMatch>& matches, 
												const std::pair<unsigned int, unsigned int>& working_views, 
												const Transform3Df & poseView1, 
												const Transform3Df & poseView2, 
												std::vector<SRef<CloudPoint>>& pcloud)
{
	pcloud.clear();
	Transform3Df poseView1Inverse = poseView1.inverse();
	Transform3Df poseView2Inverse = poseView2.inverse();
	cv::Mat Pose1 = (cv::Mat_<double>(3, 4) << poseView1Inverse(0, 0), poseView1Inverse(0, 1), poseView1Inverse(0, 2), poseView1Inverse(0, 3),
		poseView1Inverse(1, 0), poseView1Inverse(1, 1), poseView1Inverse(1, 2), poseView1Inverse(1, 3),
		poseView1Inverse(2, 0), poseView1Inverse(2, 1), poseView1Inverse(2, 2), poseView1Inverse(2, 3));



	cv::Mat Pose2 = (cv::Mat_<double>(3, 4) << poseView2Inverse(0, 0), poseView2Inverse(0, 1), poseView2Inverse(0, 2), poseView2Inverse(0, 3),
		poseView2Inverse(1, 0), poseView2Inverse(1, 1), poseView2Inverse(1, 2), poseView2Inverse(1, 3),
		poseView2Inverse(2, 0), poseView2Inverse(2, 1), poseView2Inverse(2, 2), poseView2Inverse(2, 3));

	double t = cv::getTickCount();
	// Create a vector to store the reprojection error of each triangulated 3D points
	std::vector<double> reproj_error;
	// unsigned int pts_size = pt2d_1.size();
	unsigned int pts_size = matches.size();

	// KPose 1 and KPose2 represent the transformations from 3D space to 2D image (K*[R|T]).
	cv::Mat_<double> KPose1;
	KPose1 = m_camMatrix * Pose1;
	cv::Mat_<double> KPose2;
	KPose2 = m_camMatrix * Pose2;

	Vector3f meanCamCenter((poseView1(0, 3) + poseView2(0, 3)) / 2, (poseView1(1, 3) + poseView2(1, 3)) / 2, (poseView1(2, 3) + poseView2(2, 3)) / 2);

	for (int i = 0; i < pts_size; i++) {
		cv::Point2f kp1 = cv::Point2f(keypointsView1[matches[i].getIndexInDescriptorA()].getX(), keypointsView1[matches[i].getIndexInDescriptorA()].getY());
		cv::Point3d u1(kp1.x, kp1.y, 1.0);
		// um1 represents an homogenous point in 3D camera space positionned on the image plan
		cv::Mat_<double> um1 = m_Kinv * cv::Mat_<double>(u1);
		u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

		cv::Point2f kp2 = cv::Point2f(keypointsView2[matches[i].getIndexInDescriptorB()].getX(), keypointsView2[matches[i].getIndexInDescriptorB()].getY());
		cv::Point3d u2(kp2.x, kp2.y, 1.0);
		// um1 represents an homogenous point in 3D camera space positionned on the image plan
		cv::Mat_<double> um2 = m_Kinv * cv::Mat_<double>(u2);
		u2.x = um2(0); u2.y = um2(1); u2.z = um2(2);

		// Compute the position of the 3D point projected in u1 for the camera1 with Pose1 and projected in u2 for the camera 2 with Pose 2
		cv::Mat_<double> X = iterativeLinearTriangulation(u1, Pose1, u2, Pose2);

		// Reproject this point on the image plane of the first camera
		cv::Mat_<double> xPt_img1 = KPose1 * X;				//reproject
		cv::Point2f xPt_img_1(xPt_img1(0) / xPt_img1(2), xPt_img1(1) / xPt_img1(2));

		double reprj_err1 = norm(xPt_img_1 - kp1);

		// Reproject this point on the image plane of the second camera
		cv::Mat_<double> xPt_img2 = KPose2 * X;				//reproject
		cv::Point2f xPt_img_2(xPt_img2(0) / xPt_img2(2), xPt_img2(1) / xPt_img2(2));

		double reprj_err2 = norm(xPt_img_2 - kp2);

		double reprj_err = (reprj_err1 + reprj_err2) / 2;

		reproj_error.push_back(reprj_err);

		std::map<unsigned int, unsigned int> visibility;

		visibility[working_views.first] = matches[i].getIndexInDescriptorA();
		visibility[working_views.second] = matches[i].getIndexInDescriptorB();

		// calculate the mean of two features
		cv::Mat cvDescMean;

		if (descriptor1->getDescriptorDataType() == DescriptorDataType::TYPE_8U){
			DescriptorView8U desc_1 = descriptor1->getDescriptor<DescriptorDataType::TYPE_8U>(matches[i].getIndexInDescriptorA());
			DescriptorView8U desc_2 = descriptor2->getDescriptor<DescriptorDataType::TYPE_8U>(matches[i].getIndexInDescriptorB());

			cv::Mat cvDesc1(1, desc_1.length(), desc_1.type());
			cvDesc1.data = (uchar*)desc_1.data();

			cv::Mat cvDesc2(1, desc_2.length(), desc_2.type());
			cvDesc2.data = (uchar*)desc_2.data();
			
			cvDescMean = cvDesc1 / 2 + cvDesc2 / 2;
		}
		else {
			DescriptorView32F desc_1 = descriptor1->getDescriptor<DescriptorDataType::TYPE_32F>(matches[i].getIndexInDescriptorA());
			DescriptorView32F desc_2 = descriptor2->getDescriptor<DescriptorDataType::TYPE_32F>(matches[i].getIndexInDescriptorB());

			cv::Mat cvDesc1(1, desc_1.length(), desc_1.type());
			cvDesc1.data = (uchar*)desc_1.data();

			cv::Mat cvDesc2(1, desc_2.length(), desc_2.type());
			cvDesc2.data = (uchar*)desc_2.data();

			cvDescMean = cvDesc1 / 2 + cvDesc2 / 2;
		}

		SRef<DescriptorBuffer> descMean = xpcf::utils::make_shared<DescriptorBuffer>(cvDescMean.data, descriptor1->getDescriptorType(), descriptor1->getDescriptorDataType(), descriptor1->getNbElements(), 1);

		// make a new cloud point
		SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(X(0), X(1), X(2), 0.0, 0.0, 0.0, meanCamCenter(0) - X(0), meanCamCenter(1) - X(1), meanCamCenter(2) - X(2), reprj_err, visibility, descMean);
		pcloud.push_back(cp);
	}
	cv::Scalar mse = cv::mean(reproj_error);
	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	return mse[0];
}

double SolARSVDTriangulationOpencv::triangulate(const SRef<Keyframe> & curKeyframe,
                                                const std::vector<DescriptorMatch> & matches,
                                                std::vector<SRef<CloudPoint>> & pcloud) {

    SRef<Keyframe> refKeyframe = curKeyframe->getReferenceKeyframe();

    return triangulate(refKeyframe->getKeypoints(),
                       curKeyframe->getKeypoints(),
                       refKeyframe->getDescriptors(),
                       curKeyframe->getDescriptors(),
                       matches,
                       std::make_pair(refKeyframe->getId(),curKeyframe->getId()),
                       refKeyframe->getPose(),
                       curKeyframe->getPose(),
                       pcloud);
}

void SolARSVDTriangulationOpencv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) {
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

    cv::invert(m_camMatrix,m_Kinv);
}


}
}
}
