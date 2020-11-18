

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
	declareInjectable<api::geom::IProject>(m_projector);
    LOG_DEBUG(" SolARSVDTriangulationOpencv constructor");
    m_camMatrix.create(3, 3, CV_32FC1);
    m_camDistortion.create(5, 1, CV_32FC1);
}

SolARSVDTriangulationOpencv::~SolARSVDTriangulationOpencv(){

}


cv::Mat SolARSVDTriangulationOpencv::iterativeLinearTriangulation(	const cv::Point3f & u1,
																	const cv::Mat & P1,
																	const cv::Point3f & u2,
																	const cv::Mat & P2){

	float w1 = 1, w2 = 1;
	cv::Mat_<float> X;
	for (int i = 0; i < 10; i++) {
		cv::Mat A(4, 4, CV_32F);
		A.row(0) = (u1.x * P1.row(2) - P1.row(0)) / w1;
		A.row(1) = (u1.y * P1.row(2) - P1.row(1)) / w1;
		A.row(2) = (u2.x * P2.row(2) - P2.row(0)) / w2;
		A.row(3) = (u2.y * P2.row(2) - P2.row(1)) / w2;
		cv::SVD::solveZ(A, X);

		float new_w1 = cv::Mat(P1.row(2) * X).at<float>(0);
		float new_w2 = cv::Mat(P2.row(2) * X).at<float>(0);

		if (std::abs(w1 - new_w1) <= EPSILON && std::abs(w2 - new_w2) <= EPSILON)
		{
			break;
		}

		w1 = new_w1;
		w2 = new_w2;
	}
	if (X.at<float>(3) != 0)
		X = X / X.at<float>(3);
    return X;
}

cv::Mat SolARSVDTriangulationOpencv::linearTriangulation(const cv::Point3f & u1,
                                                        const cv::Mat & P1,
                                                        const cv::Point3f & u2,
                                                        const cv::Mat & P2){

	cv::Mat A(4, 4, CV_32F);
	A.row(0) = u1.x * P1.row(2) - P1.row(0);
	A.row(1) = u1.y * P1.row(2) - P1.row(1);
	A.row(2) = u2.x * P2.row(2) - P2.row(0);
	A.row(3) = u2.y * P2.row(2) - P2.row(1);
	cv::Mat w, u, vt;
	cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
	
	cv::Mat_<float> X = vt.row(3).t();

	if (X.at<float>(3) != 0)
		X = X / X.at<float>(3);

    return X;
}

float SolARSVDTriangulationOpencv::getReprojectionErrorCloud(const std::vector<SRef<CloudPoint>> & original){
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
	cv::Mat Pose1 = (cv::Mat_<float>(3, 4) << poseView1Inverse(0, 0), poseView1Inverse(0, 1), poseView1Inverse(0, 2), poseView1Inverse(0, 3),
		poseView1Inverse(1, 0), poseView1Inverse(1, 1), poseView1Inverse(1, 2), poseView1Inverse(1, 3),
		poseView1Inverse(2, 0), poseView1Inverse(2, 1), poseView1Inverse(2, 2), poseView1Inverse(2, 3));

	cv::Mat Pose2 = (cv::Mat_<float>(3, 4) << poseView2Inverse(0, 0), poseView2Inverse(0, 1), poseView2Inverse(0, 2), poseView2Inverse(0, 3),
		poseView2Inverse(1, 0), poseView2Inverse(1, 1), poseView2Inverse(1, 2), poseView2Inverse(1, 3),
		poseView2Inverse(2, 0), poseView2Inverse(2, 1), poseView2Inverse(2, 2), poseView2Inverse(2, 3));

	// Get position of keypoints
    unsigned int pts_size = static_cast<unsigned int>(matches.size());
	std::vector<cv::Point2f> pts1, pts2;
    for (unsigned int i = 0; i < pts_size; ++i) {
		Point2Df kp1 = pointsView1[matches[i].getIndexInDescriptorA()];
		Point2Df kp2 = pointsView2[matches[i].getIndexInDescriptorB()];
		pts1.push_back(cv::Point2f(kp1.getX(), kp1.getY()));
		pts2.push_back(cv::Point2f(kp2.getX(), kp2.getY()));
	}

	// Undistort keypoints
	std::vector<cv::Point2f> ptsUn1, ptsUn2;
	cv::undistortPoints(pts1, ptsUn1, m_camMatrix, m_camDistortion);
	cv::undistortPoints(pts2, ptsUn2, m_camMatrix, m_camDistortion);

	// Mean camera center to calculate view direction
	Vector3f meanCamCenter((poseView1(0, 3) + poseView2(0, 3)) / 2, (poseView1(1, 3) + poseView2(1, 3)) / 2, (poseView1(2, 3) + poseView2(2, 3)) / 2);

	// Triangulation
	std::vector<Point3Df> pts3D;
    for (unsigned int i = 0; i < pts_size; i++) {
		cv::Point2f ptUn1 = ptsUn1[i];
		cv::Point2f ptUn2 = ptsUn2[i];
		cv::Point3f u1(ptUn1.x, ptUn1.y, 1.0);
		cv::Point3f u2(ptUn2.x, ptUn2.y, 1.0);
		cv::Mat X = iterativeLinearTriangulation(u1, Pose1, u2, Pose2);
		pts3D.push_back(Point3Df(X.at<float>(0), X.at<float>(1), X.at<float>(2)));
	}

	// Reproject 3D points
	std::vector<Point2Df> ptsIn1, ptsIn2;
	m_projector->project(pts3D, ptsIn1, poseView1);
	m_projector->project(pts3D, ptsIn2, poseView2);

	// Create cloud points
	std::vector<float> reproj_error;
    for (unsigned int i = 0; i < pts_size; ++i) {
		// Compute reprojection error
		cv::Point2f pt1 = pts1[i];
		cv::Point2f pt1_reproj = cv::Point2f(ptsIn1[i].getX(), ptsIn1[i].getY());
		cv::Point2f pt2 = pts2[i];
		cv::Point2f pt2_reproj = cv::Point2f(ptsIn2[i].getX(), ptsIn2[i].getY());
		float reprj_err = (cv::norm(pt1 - pt1_reproj) + cv::norm(pt2 - pt2_reproj)) / 2;
		reproj_error.push_back(reprj_err);
		// make visibilities
		std::map<unsigned int, unsigned int> visibility;
		visibility[working_views.first] = matches[i].getIndexInDescriptorA();
		visibility[working_views.second] = matches[i].getIndexInDescriptorB();

		// make a new cloud point
		SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(pts3D[i].getX(), pts3D[i].getY(), pts3D[i].getZ(), 0.0, 0.0, 0.0, meanCamCenter(0) - pts3D[i].getX(),
			meanCamCenter(1) - pts3D[i].getY(), meanCamCenter(2) - pts3D[i].getZ(), reprj_err, visibility);
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
	cv::Mat Pose1 = (cv::Mat_<float>(3, 4) << poseView1Inverse(0, 0), poseView1Inverse(0, 1), poseView1Inverse(0, 2), poseView1Inverse(0, 3),
		poseView1Inverse(1, 0), poseView1Inverse(1, 1), poseView1Inverse(1, 2), poseView1Inverse(1, 3),
		poseView1Inverse(2, 0), poseView1Inverse(2, 1), poseView1Inverse(2, 2), poseView1Inverse(2, 3));

	cv::Mat Pose2 = (cv::Mat_<float>(3, 4) << poseView2Inverse(0, 0), poseView2Inverse(0, 1), poseView2Inverse(0, 2), poseView2Inverse(0, 3),
		poseView2Inverse(1, 0), poseView2Inverse(1, 1), poseView2Inverse(1, 2), poseView2Inverse(1, 3),
		poseView2Inverse(2, 0), poseView2Inverse(2, 1), poseView2Inverse(2, 2), poseView2Inverse(2, 3));

	// Get position of keypoints
    unsigned int pts_size = static_cast<unsigned int>(matches.size());
	std::vector<cv::Point2f> pts1, pts2;
    for (unsigned int i = 0; i < pts_size; ++i) {
		Keypoint kp1 = keypointsView1[matches[i].getIndexInDescriptorA()];
		Keypoint kp2 = keypointsView2[matches[i].getIndexInDescriptorB()];
		pts1.push_back(cv::Point2f(kp1.getX(), kp1.getY()));
		pts2.push_back(cv::Point2f(kp2.getX(), kp2.getY()));
	}

	// Undistort keypoints
	std::vector<cv::Point2f> ptsUn1, ptsUn2;
	cv::undistortPoints(pts1, ptsUn1, m_camMatrix, m_camDistortion);
	cv::undistortPoints(pts2, ptsUn2, m_camMatrix, m_camDistortion);

	// Mean camera center to calculate view direction
	Vector3f meanCamCenter((poseView1(0, 3) + poseView2(0, 3)) / 2, (poseView1(1, 3) + poseView2(1, 3)) / 2, (poseView1(2, 3) + poseView2(2, 3)) / 2);

	// Triangulation
	std::vector<Point3Df> pts3D;
    for (unsigned int i = 0; i < pts_size; i++) {
		cv::Point2f ptUn1 = ptsUn1[i];
		cv::Point2f ptUn2 = ptsUn2[i];
		cv::Point3f u1(ptUn1.x, ptUn1.y, 1.0);
		cv::Point3f u2(ptUn2.x, ptUn2.y, 1.0);
		cv::Mat X = iterativeLinearTriangulation(u1, Pose1, u2, Pose2);
		pts3D.push_back(Point3Df(X.at<float>(0), X.at<float>(1), X.at<float>(2)));
	}

	// Reproject 3D points
	std::vector<Point2Df> ptsIn1, ptsIn2;
	m_projector->project(pts3D, ptsIn1, poseView1);
	m_projector->project(pts3D, ptsIn2, poseView2);

	// Create cloud points
	std::vector<float> reproj_error;
    for (unsigned int i = 0; i < pts_size; ++i) {
		// Compute reprojection error
		cv::Point2f pt1 = pts1[i];
		cv::Point2f pt1_reproj = cv::Point2f(ptsIn1[i].getX(), ptsIn1[i].getY());
		cv::Point2f pt2 = pts2[i];
		cv::Point2f pt2_reproj = cv::Point2f(ptsIn2[i].getX(), ptsIn2[i].getY());
		float reprj_err = (cv::norm(pt1 - pt1_reproj) + cv::norm(pt2 - pt2_reproj)) / 2;
		reproj_error.push_back(reprj_err);
		// make visibilities
		std::map<unsigned int, unsigned int> visibility;
		visibility[working_views.first] = matches[i].getIndexInDescriptorA();
		visibility[working_views.second] = matches[i].getIndexInDescriptorB();

		// make a new cloud point
		SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(pts3D[i].getX(), pts3D[i].getY(), pts3D[i].getZ(), 0.0, 0.0, 0.0, meanCamCenter(0) - pts3D[i].getX(),
			meanCamCenter(1) - pts3D[i].getY(), meanCamCenter(2) - pts3D[i].getZ(), reprj_err, visibility);
		pcloud.push_back(cp);
	}
	cv::Scalar mse = cv::mean(reproj_error);
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
	cv::Mat Pose1 = (cv::Mat_<float>(3, 4) << poseView1Inverse(0, 0), poseView1Inverse(0, 1), poseView1Inverse(0, 2), poseView1Inverse(0, 3),
		poseView1Inverse(1, 0), poseView1Inverse(1, 1), poseView1Inverse(1, 2), poseView1Inverse(1, 3),
		poseView1Inverse(2, 0), poseView1Inverse(2, 1), poseView1Inverse(2, 2), poseView1Inverse(2, 3));

	cv::Mat Pose2 = (cv::Mat_<float>(3, 4) << poseView2Inverse(0, 0), poseView2Inverse(0, 1), poseView2Inverse(0, 2), poseView2Inverse(0, 3),
		poseView2Inverse(1, 0), poseView2Inverse(1, 1), poseView2Inverse(1, 2), poseView2Inverse(1, 3),
		poseView2Inverse(2, 0), poseView2Inverse(2, 1), poseView2Inverse(2, 2), poseView2Inverse(2, 3));

	// Get position of keypoints
    unsigned int pts_size = static_cast<unsigned int>(matches.size());
	std::vector<cv::Point2f> pts1, pts2;
	for (unsigned int i = 0; i < pts_size; ++i) {
		const Keypoint &kp1 = keypointsView1[matches[i].getIndexInDescriptorA()];
		const Keypoint &kp2 = keypointsView2[matches[i].getIndexInDescriptorB()];
		pts1.push_back(cv::Point2f(kp1.getX(), kp1.getY()));
		pts2.push_back(cv::Point2f(kp2.getX(), kp2.getY()));
	}

	// Undistort keypoints
	std::vector<cv::Point2f> ptsUn1, ptsUn2;
	cv::undistortPoints(pts1, ptsUn1, m_camMatrix, m_camDistortion);
	cv::undistortPoints(pts2, ptsUn2, m_camMatrix, m_camDistortion);	

	// Mean camera center to calculate view direction
	Vector3f meanCamCenter((poseView1(0, 3) + poseView2(0, 3)) / 2, (poseView1(1, 3) + poseView2(1, 3)) / 2, (poseView1(2, 3) + poseView2(2, 3)) / 2);

	// Triangulation
	std::vector<Point3Df> pts3D;
    for (unsigned int i = 0; i < pts_size; i++) {
		cv::Point2f ptUn1 = ptsUn1[i];
		cv::Point2f ptUn2 = ptsUn2[i];
		cv::Point3f u1(ptUn1.x, ptUn1.y, 1.0);
		cv::Point3f u2(ptUn2.x, ptUn2.y, 1.0);
		cv::Mat X = iterativeLinearTriangulation(u1, Pose1, u2, Pose2);
		pts3D.push_back(Point3Df(X.at<float>(0), X.at<float>(1), X.at<float>(2)));
	}

	// Reproject 3D points
	std::vector<Point2Df> ptsIn1, ptsIn2;
	m_projector->project(pts3D, ptsIn1, poseView1);
	m_projector->project(pts3D, ptsIn2, poseView2);

	// Create cloud points
	std::vector<float> reproj_error;
    for (unsigned int i = 0; i < pts_size; ++i) {
		// Compute reprojection error
		cv::Point2f pt1 = pts1[i];
		cv::Point2f pt1_reproj = cv::Point2f(ptsIn1[i].getX(), ptsIn1[i].getY());
		cv::Point2f pt2 = pts2[i];
		cv::Point2f pt2_reproj = cv::Point2f(ptsIn2[i].getX(), ptsIn2[i].getY());
		float reprj_err = (cv::norm(pt1 - pt1_reproj) + cv::norm(pt2 - pt2_reproj)) / 2;
		reproj_error.push_back(reprj_err);
		// make visibilities
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

		// rgb mean
		const Keypoint &kp1 = keypointsView1[matches[i].getIndexInDescriptorA()];
		const Keypoint &kp2 = keypointsView2[matches[i].getIndexInDescriptorB()];
		Vector3f rgbMean = (kp1.getRGB() + kp2.getRGB()) / 2.f / 255.f;

		// view direction
		Vector3f viewNor(meanCamCenter(0) - pts3D[i].getX(), meanCamCenter(1) - pts3D[i].getY(), meanCamCenter(2) - pts3D[i].getZ());
		viewNor = viewNor / viewNor.norm();

		// make a new cloud point
		SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(pts3D[i].getX(), pts3D[i].getY(), pts3D[i].getZ(), rgbMean[0], rgbMean[1], rgbMean[2], 
			viewNor[0], viewNor[1], viewNor[2], reprj_err, visibility, descMean);
		pcloud.push_back(cp);
	}
	cv::Scalar mse = cv::mean(reproj_error);
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

void SolARSVDTriangulationOpencv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
    this->m_camDistortion.at<float>(0, 0)  = distortionParams(0);
    this->m_camDistortion.at<float>(1, 0)  = distortionParams(1);
    this->m_camDistortion.at<float>(2, 0)  = distortionParams(2);
    this->m_camDistortion.at<float>(3, 0)  = distortionParams(3);
    this->m_camDistortion.at<float>(4, 0)  = distortionParams(4);

    this->m_camMatrix.at<float>(0, 0) = intrinsicParams(0,0);
    this->m_camMatrix.at<float>(0, 1) = intrinsicParams(0,1);
    this->m_camMatrix.at<float>(0, 2) = intrinsicParams(0,2);
    this->m_camMatrix.at<float>(1, 0) = intrinsicParams(1,0);
    this->m_camMatrix.at<float>(1, 1) = intrinsicParams(1,1);
    this->m_camMatrix.at<float>(1, 2) = intrinsicParams(1,2);
    this->m_camMatrix.at<float>(2, 0) = intrinsicParams(2,0);
    this->m_camMatrix.at<float>(2, 1) = intrinsicParams(2,1);
    this->m_camMatrix.at<float>(2, 2) = intrinsicParams(2,2);

	m_projector->setCameraParameters(intrinsicParams, distortionParams);
}


}
}
}
