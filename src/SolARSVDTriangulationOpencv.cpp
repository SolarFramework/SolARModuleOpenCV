

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


cv::Mat_<double> SolARSVDTriangulationOpencv::iterativeLinearTriangulation(const cv::Point3d & u,
                                                                           const cv::Matx34d & P,
                                                                           const cv::Point3d & u1,
                                                                           const cv::Matx34d & P1){

    double wi = 1, wi1 = 1;
    cv::Mat_<double> X(4, 1);

    cv::Mat_<double> X_ = linearTriangulation(u, P, u1, P1);
    X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;

    //std::cout<<" output of linear triangulation: "<<X<<std::endl;
    for (int i = 0; i<10; i++) {
        //recalculate weights
        double p2x = cv::Mat_<double>(cv::Mat_<double>(P).row(2)*X)(0);
        double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);

        //breaking point
        if (fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

        wi = p2x;
        wi1 = p2x1;

        //reweight equations and solve
        cv::Matx43d A((u.x*P(2, 0) - P(0, 0)) / wi, (u.x*P(2, 1) - P(0, 1)) / wi, (u.x*P(2, 2) - P(0, 2)) / wi,
                      (u.y*P(2, 0) - P(1, 0)) / wi, (u.y*P(2, 1) - P(1, 1)) / wi, (u.y*P(2, 2) - P(1, 2)) / wi,
                      (u1.x*P1(2, 0) - P1(0, 0)) / wi1, (u1.x*P1(2, 1) - P1(0, 1)) / wi1, (u1.x*P1(2, 2) - P1(0, 2)) / wi1,
                      (u1.y*P1(2, 0) - P1(1, 0)) / wi1, (u1.y*P1(2, 1) - P1(1, 1)) / wi1, (u1.y*P1(2, 2) - P1(1, 2)) / wi1
                      );
        cv::Mat_<double> B = (cv::Mat_<double>(4, 1) << -(u.x*P(2, 3) - P(0, 3)) / wi,
                              -(u.y*P(2, 3) - P(1, 3)) / wi,
                              -(u1.x*P1(2, 3) - P1(0, 3)) / wi1,
                              -(u1.y*P1(2, 3) - P1(1, 3)) / wi1
                              );



        cv::solve(A, B, X_, cv::DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
        //std::cout<<"Output Non Linear Triangulation iteration "<<i<<" X: "<<X<<std::endl;
    }
    return X;

}

cv::Mat_<double> SolARSVDTriangulationOpencv::linearTriangulation(const cv::Point3d & u,
                                                                  const cv::Matx34d & P,
                                                                  const cv::Point3d & u1,
                                                                  const cv::Matx34d & P1){

    cv::Matx43d A(u.x*P(2, 0) - P(0, 0), u.x*P(2, 1) - P(0, 1), u.x*P(2, 2) - P(0, 2),
                  u.y*P(2, 0) - P(1, 0), u.y*P(2, 1) - P(1, 1), u.y*P(2, 2) - P(1, 2),
                  u1.x*P1(2, 0) - P1(0, 0), u1.x*P1(2, 1) - P1(0, 1), u1.x*P1(2, 2) - P1(0, 2),
                  u1.y*P1(2, 0) - P1(1, 0), u1.y*P1(2, 1) - P1(1, 1), u1.y*P1(2, 2) - P1(1, 2)
                  );
    cv::Matx41d B(-(u.x*P(2, 3) - P(0, 3)),
                  -(u.y*P(2, 3) - P(1, 3)),
                  -(u1.x*P1(2, 3) - P1(0, 3)),
                  -(u1.y*P1(2, 3) - P1(1, 3)));

    cv::Mat_<double> X;
    cv::solve(A, B, X, cv::DECOMP_SVD);
    return X;
}

/**
From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
*/
cv::Mat_<double> SolARSVDTriangulationOpencv::LinearLSTriangulation(const cv::Point2d & u,       //homogenous image point (u,v,1)
                                                                    const cv::Matx34d & P,       //camera 1 matrix
                                                                    const cv::Point2d & u1,      //homogenous image point in 2nd camera
                                                                    const cv::Matx34d & P1,       //camera 2 matrix
                                                                    double & error){
    cv::Matx<double, 4, 4> matrA;
    cv::Matx<double, 4, 4> matrU;
    cv::Matx<double, 4, 1> matrW;
    cv::Matx<double, 4, 4> matrV;

    double x = u.x;
    double y = u.y;
    double x1 = u1.x;
    double y1 = u1.y;

    for (int k = 0; k < 4; k++)
    {
        matrA(0, k) = x * P(2, k) - P(0, k);
        matrA(1, k) = y * P(2, k) - P(1, k);
        matrA(2, k) = x1 * P1(2, k) - P1(0, k);
        matrA(3, k) = y1 * P1(2, k) - P1(1, k);
    }

    cv::SVD::compute(matrA, matrW, matrU, matrV);
    cv::Mat X(4, 1, CV_64FC1);
    X.at<double>(0, 0) = matrV(3, 0);
    X.at<double>(1, 0) = matrV(3, 1);
    X.at<double>(2, 0) = matrV(3, 2);
    X.at<double>(3, 0) = matrV(3, 3);

    // calculate error project
    cv::Mat reproj1 = cv::Mat(P) * X;
    cv::Point2d pts_reproj1(reproj1.at<double>(0, 0) / reproj1.at<double>(2, 0), reproj1.at<double>(1, 0) / reproj1.at<double>(2, 0));

    cv::Mat reproj2 = cv::Mat(P1) * X;
    cv::Point2d pts_reproj2(reproj2.at<double>(0, 0) / reproj2.at<double>(2, 0), reproj2.at<double>(1, 0) / reproj2.at<double>(2, 0));

    error = (cv::norm(pts_reproj1 - u) + cv::norm(pts_reproj2 - u1)) / 2;

    //out = cv::Vec3f((float)matrV(3, 0) / matrV(3, 3), (float)matrV(3, 1) / matrV(3, 3), (float)matrV(3, 2) / matrV(3, 3));
    cv::Mat_<double> output(matrV(3, 0) / matrV(3, 3), matrV(3, 1) / matrV(3, 3), matrV(3, 2) / matrV(3, 3));
    return output;
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
    
    cv::Matx34d Pose1( poseView1Inverse(0, 0),poseView1Inverse(0, 1),poseView1Inverse(0, 2), poseView1Inverse(0, 3),
                       poseView1Inverse(1, 0),poseView1Inverse(1, 1),poseView1Inverse(1, 2), poseView1Inverse(1, 3),
                       poseView1Inverse(2, 0),poseView1Inverse(2, 1),poseView1Inverse(2, 2), poseView1Inverse(2, 3));



    cv::Matx34d Pose2(poseView2Inverse(0, 0),poseView2Inverse(0, 1),poseView2Inverse(0, 2), poseView2Inverse(0, 3),
                      poseView2Inverse(1, 0),poseView2Inverse(1, 1),poseView2Inverse(1, 2), poseView2Inverse(1, 3),
                      poseView2Inverse(2, 0),poseView2Inverse(2, 1),poseView2Inverse(2, 2), poseView2Inverse(2, 3));

    cv::Mat_<double> Kinv;

    cv::invert(m_camMatrix,Kinv);
    // Create a vector to store the reprojection error of each triangulated 3D points
    std::vector<double> reproj_error;
    // unsigned int pts_size = pt2d_1.size();
    unsigned int pts_size = matches.size();

    // KPose 1 and KPose2 represent the transformations from 3D space to 2D image (K*[R|T]).
    cv::Mat_<double> KPose1;
    KPose1 = m_camMatrix * cv::Mat(Pose1);
	Vector3f meanCamCenter((poseView1(0, 3) + poseView2(0, 3)) / 2, (poseView1(1, 3) + poseView2(1, 3)) / 2, (poseView1(2, 3) + poseView2(2, 3)) / 2);

    for (int i = 0; i<pts_size; i++) {
        cv::Point2f kp1 = cv::Point2f(pointsView1[matches[i].getIndexInDescriptorA()].getX(),pointsView1[matches[i].getIndexInDescriptorA()].getY());
        cv::Point3d u1(kp1.x, kp1.y, 1.0);
        // um1 represents an homogenous point in 3D camera space positionned on the image plan
        cv::Mat_<double> um1 = Kinv * cv::Mat_<double>(u1);
        u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

        cv::Point2f kp2 = cv::Point2f(pointsView2[matches[i].getIndexInDescriptorB()].getX(),pointsView2[matches[i].getIndexInDescriptorB()].getY());
        cv::Point3d u2(kp2.x, kp2.y, 1.0);
        // um1 represents an homogenous point in 3D camera space positionned on the image plan
        cv::Mat_<double> um2 = Kinv * cv::Mat_<double>(u2);
        u2.x = um2(0); u2.y = um2(1); u2.z = um2(2);

        // Compute the position of the 3D point projected in u1 for the camera1 with Pose1 and projected in u2 for the camera 2 with Pose 2
        //cv::Mat_<double> X = iterativeLinearTriangulation(u1, Pose1, u2, Pose2);
        cv::Mat_<double> X = iterativeLinearTriangulation(u1, Pose1, u2, Pose2);

        //std::cout<<"X: "<<X<<std::endl;

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
    cv::Matx34d Pose1( poseView1Inverse(0, 0),poseView1Inverse(0, 1),poseView1Inverse(0, 2), poseView1Inverse(0, 3),
                       poseView1Inverse(1, 0),poseView1Inverse(1, 1),poseView1Inverse(1, 2), poseView1Inverse(1, 3),
                       poseView1Inverse(2, 0),poseView1Inverse(2, 1),poseView1Inverse(2, 2), poseView1Inverse(2, 3));



    cv::Matx34d Pose2(poseView2Inverse(0, 0),poseView2Inverse(0, 1),poseView2Inverse(0, 2), poseView2Inverse(0, 3),
                      poseView2Inverse(1, 0),poseView2Inverse(1, 1),poseView2Inverse(1, 2), poseView2Inverse(1, 3),
                      poseView2Inverse(2, 0),poseView2Inverse(2, 1),poseView2Inverse(2, 2), poseView2Inverse(2, 3));

    cv::Mat_<double> Kinv;

    cv::invert(m_camMatrix,Kinv);
    //std::cout<<"KInv :" << Kinv <<std::endl;
    double t = cv::getTickCount();
    // Create a vector to store the reprojection error of each triangulated 3D points
    std::vector<double> reproj_error;
    // unsigned int pts_size = pt2d_1.size();
    unsigned int pts_size = matches.size();

    // KPose 1 and KPose2 represent the transformations from 3D space to 2D image (K*[R|T]).
    cv::Mat_<double> KPose1;
    KPose1 = m_camMatrix * cv::Mat(Pose1);
	Vector3f meanCamCenter((poseView1(0, 3) + poseView2(0, 3)) / 2, (poseView1(1, 3) + poseView2(1, 3)) / 2, (poseView1(2, 3) + poseView2(2, 3)) / 2);

    for (int i = 0; i<pts_size; i++) {
        cv::Point2f kp1 = cv::Point2f(keypointsView1[matches[i].getIndexInDescriptorA()].getX(), keypointsView1[matches[i].getIndexInDescriptorA()].getY());
        cv::Point3d u1(kp1.x, kp1.y, 1.0);
        // um1 represents an homogenous point in 3D camera space positionned on the image plan
        cv::Mat_<double> um1 = Kinv * cv::Mat_<double>(u1);
        u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

        cv::Point2f kp2 = cv::Point2f(keypointsView2[matches[i].getIndexInDescriptorB()].getX(), keypointsView2[matches[i].getIndexInDescriptorB()].getY());
        cv::Point3d u2(kp2.x, kp2.y, 1.0);
        // um1 represents an homogenous point in 3D camera space positionned on the image plan
        cv::Mat_<double> um2 = Kinv * cv::Mat_<double>(u2);
        u2.x = um2(0); u2.y = um2(1); u2.z = um2(2);

        //std::cout<<"point1: "<< kp1 <<", u1: "<<u1<<std::endl;
        //std::cout<<"P1: "<<Pose1<<std::endl;

        //std::cout<<"point2: "<< kp2 <<", u2: "<<u2<<std::endl;
        //std::cout<<"P2: "<<Pose2<<std::endl;

        // Compute the position of the 3D point projected in u1 for the camera1 with Pose1 and projected in u2 for the camera 2 with Pose 2

        cv::Mat_<double> X = iterativeLinearTriangulation(u1, Pose1, u2, Pose2);
        //double error;


        //std::cout<<"X: "<<X<<std::endl;

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
	cv::Matx34d Pose1(poseView1Inverse(0, 0), poseView1Inverse(0, 1), poseView1Inverse(0, 2), poseView1Inverse(0, 3),
		poseView1Inverse(1, 0), poseView1Inverse(1, 1), poseView1Inverse(1, 2), poseView1Inverse(1, 3),
		poseView1Inverse(2, 0), poseView1Inverse(2, 1), poseView1Inverse(2, 2), poseView1Inverse(2, 3));



	cv::Matx34d Pose2(poseView2Inverse(0, 0), poseView2Inverse(0, 1), poseView2Inverse(0, 2), poseView2Inverse(0, 3),
		poseView2Inverse(1, 0), poseView2Inverse(1, 1), poseView2Inverse(1, 2), poseView2Inverse(1, 3),
		poseView2Inverse(2, 0), poseView2Inverse(2, 1), poseView2Inverse(2, 2), poseView2Inverse(2, 3));

	cv::Mat_<double> Kinv;

	cv::invert(m_camMatrix, Kinv);
	//std::cout<<"KInv :" << Kinv <<std::endl;
	double t = cv::getTickCount();
	// Create a vector to store the reprojection error of each triangulated 3D points
	std::vector<double> reproj_error;
	// unsigned int pts_size = pt2d_1.size();
	unsigned int pts_size = matches.size();

	// KPose 1 and KPose2 represent the transformations from 3D space to 2D image (K*[R|T]).
	cv::Mat_<double> KPose1;
	KPose1 = m_camMatrix * cv::Mat(Pose1);
	cv::Mat_<double> KPose2;
	KPose2 = m_camMatrix * cv::Mat(Pose2);

	Vector3f meanCamCenter((poseView1(0, 3) + poseView2(0, 3)) / 2, (poseView1(1, 3) + poseView2(1, 3)) / 2, (poseView1(2, 3) + poseView2(2, 3)) / 2);

	for (int i = 0; i < pts_size; i++) {
		cv::Point2f kp1 = cv::Point2f(keypointsView1[matches[i].getIndexInDescriptorA()].getX(), keypointsView1[matches[i].getIndexInDescriptorA()].getY());
		cv::Point3d u1(kp1.x, kp1.y, 1.0);
		// um1 represents an homogenous point in 3D camera space positionned on the image plan
		cv::Mat_<double> um1 = Kinv * cv::Mat_<double>(u1);
		u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

		cv::Point2f kp2 = cv::Point2f(keypointsView2[matches[i].getIndexInDescriptorB()].getX(), keypointsView2[matches[i].getIndexInDescriptorB()].getY());
		cv::Point3d u2(kp2.x, kp2.y, 1.0);
		// um1 represents an homogenous point in 3D camera space positionned on the image plan
		cv::Mat_<double> um2 = Kinv * cv::Mat_<double>(u2);
		u2.x = um2(0); u2.y = um2(1); u2.z = um2(2);

		//std::cout<<"point1: "<< kp1 <<", u1: "<<u1<<std::endl;
		//std::cout<<"P1: "<<Pose1<<std::endl;

		//std::cout<<"point2: "<< kp2 <<", u2: "<<u2<<std::endl;
		//std::cout<<"P2: "<<Pose2<<std::endl;

		// Compute the position of the 3D point projected in u1 for the camera1 with Pose1 and projected in u2 for the camera 2 with Pose 2

		cv::Mat_<double> X = iterativeLinearTriangulation(u1, Pose1, u2, Pose2);
		//double error;


		//std::cout<<"X: "<<X<<std::endl;

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
    //TODO.. check to inverse
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
