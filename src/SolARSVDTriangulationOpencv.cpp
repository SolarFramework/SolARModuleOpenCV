

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

Point3Df SolARSVDTriangulationOpencv::unproject3DPoint(const Keypoint & kp, const Transform3Df & pose)
{
	float Z = kp.getDepth();
	float X = (kp.getX() - m_intrinsicParams(0, 2)) * Z / m_intrinsicParams(0, 0);
	float Y = (kp.getY() - m_intrinsicParams(1, 2)) * Z / m_intrinsicParams(1, 1);
	Vector3f pos3D(X, Y, Z);
	Vector3f pos3DTrans = pose * pos3D;
	return Point3Df(pos3DTrans[0], pos3DTrans[1], pos3DTrans[2]);
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
                                                const std::pair<uint32_t,uint32_t> & working_views,
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
    uint32_t pts_size = static_cast<uint32_t>(matches.size());
	std::vector<cv::Point2f> pts1, pts2;
    for (uint32_t i = 0; i < pts_size; ++i) {
		Point2Df kp1 = pointsView1[matches[i].getIndexInDescriptorA()];
		Point2Df kp2 = pointsView2[matches[i].getIndexInDescriptorB()];
		pts1.push_back(cv::Point2f(kp1.getX(), kp1.getY()));
		pts2.push_back(cv::Point2f(kp2.getX(), kp2.getY()));
	}

	// Mean camera center to calculate view direction
	Vector3f meanCamCenter((poseView1(0, 3) + poseView2(0, 3)) / 2, (poseView1(1, 3) + poseView2(1, 3)) / 2, (poseView1(2, 3) + poseView2(2, 3)) / 2);

	// Triangulation
	std::vector<Point3Df> pts3D;
    for (uint32_t i = 0; i < pts_size; i++) {
		cv::Point2f ptUn1 = pts1[i];
		cv::Point2f ptUn2 = pts2[i];
		cv::Point3f u1((ptUn1.x - m_intrinsicParams(0, 2)) / m_intrinsicParams(0, 0), (ptUn1.y - m_intrinsicParams(1, 2)) / m_intrinsicParams(1, 1), 1.0);
		cv::Point3f u2((ptUn2.x - m_intrinsicParams(0, 2)) / m_intrinsicParams(0, 0), (ptUn2.y - m_intrinsicParams(1, 2)) / m_intrinsicParams(1, 1), 1.0);
		cv::Mat X = iterativeLinearTriangulation(u1, Pose1, u2, Pose2);
		pts3D.push_back(Point3Df(X.at<float>(0), X.at<float>(1), X.at<float>(2)));
	}

	// Reproject 3D points
	std::vector<Point2Df> ptsIn1, ptsIn2;
	m_projector->project(pts3D, ptsIn1, poseView1);
	m_projector->project(pts3D, ptsIn2, poseView2);

	// Create cloud points
	std::vector<float> reproj_error;
    for (uint32_t i = 0; i < pts_size; ++i) {
		// Compute reprojection error
		cv::Point2f pt1 = pts1[i];
		cv::Point2f pt1_reproj = cv::Point2f(ptsIn1[i].getX(), ptsIn1[i].getY());
		cv::Point2f pt2 = pts2[i];
		cv::Point2f pt2_reproj = cv::Point2f(ptsIn2[i].getX(), ptsIn2[i].getY());
		float reprj_err = (cv::norm(pt1 - pt1_reproj) + cv::norm(pt2 - pt2_reproj)) / 2;
		reproj_error.push_back(reprj_err);
		// make visibilities
		std::map<uint32_t, uint32_t> visibility;
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
                                                const std::pair<uint32_t,uint32_t> & working_views,
                                                const Transform3Df & poseView1,
                                                const Transform3Df & poseView2,
                                                std::vector<SRef<CloudPoint>> & pcloud){
	// Get position of keypoints
	std::vector<Point2Df> pointsView1, pointsView2;
	for (const auto & kp : keypointsView1)
		pointsView1.push_back(Point2Df(kp));
	for (const auto & kp : keypointsView2)
		pointsView2.push_back(Point2Df(kp));

	// triangulate
	double meanError = this->triangulate(pointsView1, pointsView2, matches, working_views, poseView1, poseView2, pcloud);
	assert(pcloud.size() == matches.size());

	// compute RGB for each cloud point
    for (uint32_t i = 0; i < matches.size(); ++i) {
		const Keypoint &kp1 = keypointsView1[matches[i].getIndexInDescriptorA()];
		const Keypoint &kp2 = keypointsView2[matches[i].getIndexInDescriptorB()];
		Vector3f rgbMean = (kp1.getRGB() + kp2.getRGB()) / 2.f / 255.f;
		pcloud[i]->setRGB(rgbMean);
	}
	return meanError;
}

double SolARSVDTriangulationOpencv::triangulate(const std::vector<Keypoint>& keypointsView1, 
												const std::vector<Keypoint>& keypointsView2, 
												const SRef<DescriptorBuffer>& descriptor1,
												const SRef<DescriptorBuffer>& descriptor2,
												const std::vector<DescriptorMatch>& matches, 
												const std::pair<uint32_t, uint32_t>& working_views, 
												const Transform3Df & poseView1, 
												const Transform3Df & poseView2, 
												std::vector<SRef<CloudPoint>>& pcloud)
{
	// triangulate
	double meanError = this->triangulate(keypointsView1, keypointsView2, matches, working_views, poseView1, poseView2, pcloud);
	assert(pcloud.size() == matches.size());

	// compute descriptor for each cloud point
	for (uint32_t i = 0; i < matches.size(); ++i) {
		cv::Mat cvDescMean;
		if (descriptor1->getDescriptorDataType() == DescriptorDataType::TYPE_8U){
			DescriptorView8U desc_1 = descriptor1->getDescriptor<DescriptorDataType::TYPE_8U>(matches[i].getIndexInDescriptorA());
			DescriptorView8U desc_2 = descriptor2->getDescriptor<DescriptorDataType::TYPE_8U>(matches[i].getIndexInDescriptorB());

			cv::Mat cvDesc1(1, desc_1.length(), CV_8U);
			cvDesc1.data = (uchar*)desc_1.data();

			cv::Mat cvDesc2(1, desc_2.length(), CV_8U);
			cvDesc2.data = (uchar*)desc_2.data();
			
			cvDescMean = cvDesc1 / 2 + cvDesc2 / 2;
		}
		else {
			DescriptorView32F desc_1 = descriptor1->getDescriptor<DescriptorDataType::TYPE_32F>(matches[i].getIndexInDescriptorA());
			DescriptorView32F desc_2 = descriptor2->getDescriptor<DescriptorDataType::TYPE_32F>(matches[i].getIndexInDescriptorB());

			cv::Mat cvDesc1(1, desc_1.length(), CV_32F);
			cvDesc1.data = (uchar*)desc_1.data();

			cv::Mat cvDesc2(1, desc_2.length(), CV_32F);
			cvDesc2.data = (uchar*)desc_2.data();

			cvDescMean = cvDesc1 / 2 + cvDesc2 / 2;
		}
		SRef<DescriptorBuffer> descMean = xpcf::utils::make_shared<DescriptorBuffer>(cvDescMean.data, descriptor1->getDescriptorType(), descriptor1->getDescriptorDataType(), descriptor1->getNbElements(), 1);
		pcloud[i]->setDescriptor(descMean);
	}
	return meanError;
}

double SolARSVDTriangulationOpencv::triangulate(SRef<SolAR::datastructure::Frame> frame1, SRef<SolAR::datastructure::Frame> frame2, const std::vector<SolAR::datastructure::DescriptorMatch>& matches, const std::pair<uint32_t, uint32_t>& working_views, std::vector<SRef<SolAR::datastructure::CloudPoint>>& pcloud, const bool & onlyDepth)
{
	pcloud.clear();
	Transform3Df poseView1 = frame1->getPose();
	Transform3Df poseView2 = frame2->getPose();
	Transform3Df poseView1Inverse = poseView1.inverse();
	Transform3Df poseView2Inverse = poseView2.inverse();
	cv::Mat Pose1 = (cv::Mat_<float>(3, 4) << poseView1Inverse(0, 0), poseView1Inverse(0, 1), poseView1Inverse(0, 2), poseView1Inverse(0, 3),
		poseView1Inverse(1, 0), poseView1Inverse(1, 1), poseView1Inverse(1, 2), poseView1Inverse(1, 3),
		poseView1Inverse(2, 0), poseView1Inverse(2, 1), poseView1Inverse(2, 2), poseView1Inverse(2, 3));

	cv::Mat Pose2 = (cv::Mat_<float>(3, 4) << poseView2Inverse(0, 0), poseView2Inverse(0, 1), poseView2Inverse(0, 2), poseView2Inverse(0, 3),
		poseView2Inverse(1, 0), poseView2Inverse(1, 1), poseView2Inverse(1, 2), poseView2Inverse(1, 3),
		poseView2Inverse(2, 0), poseView2Inverse(2, 1), poseView2Inverse(2, 2), poseView2Inverse(2, 3));

	const SRef<DescriptorBuffer>& descriptor1 = frame1->getDescriptors();
	const SRef<DescriptorBuffer>& descriptor2 = frame2->getDescriptors();
	const std::vector<Keypoint>& kpsUn1 = frame1->getUndistortedKeypoints();
	const std::vector<Keypoint>& kpsUn2 = frame2->getUndistortedKeypoints();
	std::vector<SolAR::datastructure::DescriptorMatch> goodMatches;
	std::vector<Point3Df> pts3D;
	for (int i = 0; i < matches.size(); ++i) {
		const Keypoint &kp1 = kpsUn1[matches[i].getIndexInDescriptorA()];
		const Keypoint &kp2 = kpsUn2[matches[i].getIndexInDescriptorB()];
		Point3Df pt3D;		
		if (kp1.getDepth() > 0) {	// using only depth of keypoint1
			pt3D = unproject3DPoint(kp1, poseView1);
		}
		else if (kp2.getDepth() > 0) {	// using only depth of keypoint2
			pt3D = unproject3DPoint(kp2, poseView2);
		}
		else if (!onlyDepth) {	// triangulation
			cv::Point3f u1((kp1.getX() - m_intrinsicParams(0, 2)) / m_intrinsicParams(0, 0), (kp1.getY() - m_intrinsicParams(1, 2)) / m_intrinsicParams(1, 1), 1.0);
			cv::Point3f u2((kp2.getX() - m_intrinsicParams(0, 2)) / m_intrinsicParams(0, 0), (kp2.getY() - m_intrinsicParams(1, 2)) / m_intrinsicParams(1, 1), 1.0);
			cv::Mat X = iterativeLinearTriangulation(u1, Pose1, u2, Pose2);
			pt3D = Point3Df(X.at<float>(0), X.at<float>(1), X.at<float>(2));
		}
		else
			continue;
		pts3D.push_back(pt3D);
		goodMatches.push_back(matches[i]);
	}
	if (pts3D.size() == 0)
		return 0.0;
	// Reproject 3D points
	std::vector<Point2Df> ptsIn1, ptsIn2;
	m_projector->project(pts3D, ptsIn1, poseView1);
	m_projector->project(pts3D, ptsIn2, poseView2);

	// Mean camera center to calculate view direction
	Vector3f meanCamCenter((poseView1(0, 3) + poseView2(0, 3)) / 2, (poseView1(1, 3) + poseView2(1, 3)) / 2, (poseView1(2, 3) + poseView2(2, 3)) / 2);

	// Create cloud points
	std::vector<float> reproj_error;
	for (uint32_t i = 0; i < goodMatches.size(); ++i) {
		// Compute reprojection error
		const Keypoint& kpUn1 = kpsUn1[goodMatches[i].getIndexInDescriptorA()];
		const Keypoint& kpUn2 = kpsUn2[goodMatches[i].getIndexInDescriptorB()];
		float reprj_err = ((kpUn1 - ptsIn1[i]).norm() + (kpUn2 - ptsIn2[i]).norm()) / 2;
		reproj_error.push_back(reprj_err);

		// make visibilities
		std::map<uint32_t, uint32_t> visibility;
		visibility[working_views.first] = goodMatches[i].getIndexInDescriptorA();
		visibility[working_views.second] = goodMatches[i].getIndexInDescriptorB();

		// calculate RGB
		Vector3f rgbMean = (kpUn1.getRGB() + kpUn2.getRGB()) / 2.f / 255.f;

		// calculate descriptor
		cv::Mat cvDescMean;
		if (descriptor1->getDescriptorDataType() == DescriptorDataType::TYPE_8U) {
			DescriptorView8U desc_1 = descriptor1->getDescriptor<DescriptorDataType::TYPE_8U>(goodMatches[i].getIndexInDescriptorA());
			DescriptorView8U desc_2 = descriptor2->getDescriptor<DescriptorDataType::TYPE_8U>(goodMatches[i].getIndexInDescriptorB());
			cv::Mat cvDesc1(1, desc_1.length(), CV_8U);
			cvDesc1.data = (uchar*)desc_1.data();
			cv::Mat cvDesc2(1, desc_2.length(), CV_8U);
			cvDesc2.data = (uchar*)desc_2.data();
			cvDescMean = cvDesc1 / 2 + cvDesc2 / 2;
		}
		else {
			DescriptorView32F desc_1 = descriptor1->getDescriptor<DescriptorDataType::TYPE_32F>(goodMatches[i].getIndexInDescriptorA());
			DescriptorView32F desc_2 = descriptor2->getDescriptor<DescriptorDataType::TYPE_32F>(goodMatches[i].getIndexInDescriptorB());
			cv::Mat cvDesc1(1, desc_1.length(), CV_32F);
			cvDesc1.data = (uchar*)desc_1.data();
			cv::Mat cvDesc2(1, desc_2.length(), CV_32F);
			cvDesc2.data = (uchar*)desc_2.data();
			cvDescMean = cvDesc1 / 2 + cvDesc2 / 2;
		}
		SRef<DescriptorBuffer> descMean = xpcf::utils::make_shared<DescriptorBuffer>(cvDescMean.data, descriptor1->getDescriptorType(), descriptor1->getDescriptorDataType(), descriptor1->getNbElements(), 1);

		// make a new cloud point
		SRef<CloudPoint> cp = xpcf::utils::make_shared<CloudPoint>(pts3D[i].getX(), pts3D[i].getY(), pts3D[i].getZ(), 
			rgbMean[0], rgbMean[1], rgbMean[2], meanCamCenter(0) - pts3D[i].getX(), meanCamCenter(1) - pts3D[i].getY(), meanCamCenter(2) - pts3D[i].getZ(), 
			reprj_err, visibility, descMean);
		pcloud.push_back(cp);
	}
	cv::Scalar mse = cv::mean(reproj_error);
	return mse[0];
}

void SolARSVDTriangulationOpencv::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
	m_intrinsicParams = intrinsicParams;
	m_distortionParams = distortionParams;
	// set zero distortion for projector
	m_projector->setCameraParameters(intrinsicParams, CamDistortion::Zero());
}


}
}
}
