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

#include "SolARPoseEstimationPnPL.h"
#include "core/Log.h"
#include <random>
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/hal/hal.hpp"
#include "opencv2/core/eigen.hpp"
#include "boost/algorithm/string.hpp"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENCV::SolARPoseEstimationPnPL);

namespace xpcf = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENCV {

	SolARPoseEstimationPnPL::SolARPoseEstimationPnPL() : ConfigurableBase(xpcf::toUUID<SolARPoseEstimationPnPL>())
	{
		declareInterface<api::solver::pose::I3DTransformFinderFrom2D3DPointLine>(this);

		m_camMatrix.create(3, 3, CV_32F);
		m_Kinv.create(3, 3, CV_32F);
		m_camDistortion.create(5, 1, CV_32F);
	}

	SolARPoseEstimationPnPL::~SolARPoseEstimationPnPL() { }

	FrameworkReturnCode SolARPoseEstimationPnPL::estimate( const std::vector<Point2Df>& imagePoints,
															const std::vector<Point3Df>& worldPoints,
															const std::vector<Edge2Df>& imageLines,
															const std::vector<Edge3Df>& worldLines,
															Transform3Df & pose,
															const Transform3Df & initialPose)
	{
		if (worldPoints.size() != imagePoints.size() ||
			worldLines.size()  != imageLines.size()  ||
			worldPoints.size() + worldLines.size() < 6)
		{
			LOG_WARNING("world/image points and lines sizes must be valid ( equals and more than 6 in total)");
			return FrameworkReturnCode::_ERROR_; // vector of 2D and 3D points must have same size
		}
		// Normalize 3D lines
		std::vector<Edge3Df> lines3D = worldLines;
		Point3Df Xd, Xe;
		for (unsigned i = 0 ; i < lines3D.size(); i++)
		{
			Xd = lines3D[i].p2 - lines3D[i].p1;
			Xd *= 1.0f / Xd.magnitude();
			Xe = lines3D[i].p1 + Xd;
			lines3D[i].p2 = Xe;
		}

		cv::Mat R0, t0;
		if (!pnpl_main(worldPoints, imagePoints, worldLines, imageLines, R0, t0))
			return FrameworkReturnCode::_ERROR_;
		
		for (int row = 0; row < 3; row++)
		{
			for (int col = 0; col < 3; col++)
				pose(row, col) = R0.at<float>(row, col);
			pose(row, 3) = t0.at<float>(row);
		}
		pose = pose.inverse();

		return FrameworkReturnCode::_SUCCESS;
	}

	FrameworkReturnCode SolARPoseEstimationPnPL::estimateRansac(	const std::vector<Point2Df>& imagePoints,
																	const std::vector<Point3Df>& worldPoints,
																	const std::vector<Edge2Df>& imageLines,
																	const std::vector<Edge3Df>& worldLines,
																	std::vector<Point2Df>& imagePoints_inliers,
																	std::vector<Point3Df>& worldPoints_inliers,
																	std::vector<Edge2Df>& imageLines_inliers,
																	std::vector<Edge3Df>& worldLines_inliers,
																	std::vector<bool>& pointInliers,
																	std::vector<bool>& lineInliers,
																	Transform3Df & pose,
																	const Transform3Df & initialPose)
	{
		// RANSAC params
		int iterationsCount = 20;
		float maxReprojError = 1.f;
		//float confidence = 0.99f;
		int minNbInliers = 50;
		int minRequiredData = 6;

		auto rng = std::default_random_engine{};

		std::vector<Point2Df> pt2dInliers;
		std::vector<Point3Df> pt3dInliers;
		std::vector<Edge2Df> ln2dInliers;
		std::vector<Edge3Df> ln3dInliers;

		int it = 0;
		float bestReprojError(INFINITY);
		bool success(false);
		float error, global_error;
		cv::Mat R0, t0;
		cv::Mat R(3, 3, CV_32F);
		cv::Mat t(3, 1, CV_32F);
		std::vector<unsigned int> indicesPt(imagePoints.size()), indicesLn(imageLines.size());
		Point2Df pt2D; Point3Df pt3D;
		Edge2Df ln2D; Edge3Df ln3D;
		while (it++ < iterationsCount)
		{
			if (it % 10 == 0)
				LOG_DEBUG("PnPL it: {}", it);
			// Select n random values from data
			std::iota(indicesPt.begin(), indicesPt.end(), 0);
			std::iota(indicesLn.begin(), indicesLn.end(), 0);
			std::shuffle(indicesPt.begin(), indicesPt.end(), rng);
			std::shuffle(indicesLn.begin(), indicesLn.end(), rng);
			for (int i = 0; i < minRequiredData; i++)
			{
				pt2dInliers.push_back(imagePoints[indicesPt[i]]);
				pt3dInliers.push_back(worldPoints[indicesPt[i]]);
				ln2dInliers.push_back(imageLines[indicesLn[i]]);
				ln3dInliers.push_back(worldLines[indicesLn[i]]);
			}
			// Estimate pose
			if (!pnpl_main(pt3dInliers, pt2dInliers, ln3dInliers, ln2dInliers, R, t))
				continue;
			// Compute reproj_error for each point/line
			for (int i = minRequiredData; i < imagePoints.size(); i++)
			{
				pt2D = imagePoints[indicesPt[i]];
				pt3D = worldPoints[indicesPt[i]];
				float error = getPointReprojError(pt2D, pt3D, R, t);
				if (error < maxReprojError)
				{
					pt2dInliers.push_back(pt2D);
					pt3dInliers.push_back(pt3D);
				}
			}
			for (int i = minRequiredData; i < imageLines.size(); i++)
			{
				ln2D = imageLines[indicesLn[i]];
				ln3D = worldLines[indicesLn[i]];
				error = getLineReprojError(imageLines[indicesLn[i]], worldLines[indicesLn[i]], R, t);
				if (error < maxReprojError)
				{
					ln2dInliers.push_back(ln2D);
					ln3dInliers.push_back(ln3D);
				}
			}
			// Check total point and line inliers
			if (pt2dInliers.size() + ln2dInliers.size() > minNbInliers)
			{
				// Estimate pose with all potential inliers
				if (!pnpl_main(pt3dInliers, pt2dInliers, ln3dInliers, ln2dInliers, R, t))
					continue;
				// Compute reproj error for other inliers
				global_error = 0.f;
				for (int i = 0; i < pt2dInliers.size(); i++)
					global_error += getPointReprojError(pt2dInliers[i], pt3dInliers[i], R, t);
				for (int i = 0; i < ln2dInliers.size(); i++)
					global_error += getLineReprojError(ln2dInliers[i], ln3dInliers[i], R, t);
				global_error /= float(pt2dInliers.size() + ln2dInliers.size());
				// Check against best pose
				if (global_error < bestReprojError)
				{
					success = true;
					bestReprojError = global_error;
					R0 = R;
					t0 = t;
					LOG_DEBUG("new R0 {}", R0);
					LOG_DEBUG("new t0 {}", t0);
				}
			}
		}

		LOG_DEBUG("bestReprojError: {}", bestReprojError);

		if (!success)
		{
			LOG_WARNING("Not enough inliers to estimate pose with RANSAC");
			return FrameworkReturnCode::_ERROR_;
		}

		// Retrieve final inliers
		imagePoints_inliers.clear();
		worldPoints_inliers.clear();
		imageLines_inliers.clear();
		worldLines_inliers.clear();
		pointInliers.clear();
		lineInliers.clear();

		if (!pnpl_main(worldPoints, imagePoints, worldLines, imageLines, R0, t0))
			return FrameworkReturnCode::_ERROR_;

		for (int i = 0; i < imagePoints.size(); i++)
		{
			pt2D = imagePoints[i];
			pt3D = worldPoints[i];
			error = getPointReprojError(pt2D, pt3D, R0, t0);
			pointInliers.push_back(error < maxReprojError);
			if (pointInliers[i])
			{
				imagePoints_inliers.push_back(pt2D);
				worldPoints_inliers.push_back(pt3D);
			}
		}
		for (int i = 0; i < imageLines.size(); i++)
		{
			ln2D = imageLines[i];
			ln3D = worldLines[i];
			error = getLineReprojError(ln2D, ln3D, R0, t0);
			lineInliers.push_back(error < maxReprojError);
			if (lineInliers[i])
			{
				imageLines_inliers.push_back(ln2D);
				worldLines_inliers.push_back(ln3D);
			}
		}
		if (imagePoints_inliers.size() + imageLines_inliers.size() < minNbInliers)
		{
			LOG_WARNING("world/image inliers points must be valid ( equal and > to {}): {} inliers for {} input points", std::max(3, minNbInliers), imagePoints_inliers.size() + imageLines_inliers.size(), imagePoints.size() + imageLines.size());
			return FrameworkReturnCode::_ERROR_;
		}

		// Compute pose using only inliers
		if (!pnpl_main(worldPoints_inliers, imagePoints_inliers, worldLines_inliers, imageLines_inliers, R0, t0))
			return FrameworkReturnCode::_ERROR_;
		
		for (int row = 0; row < 3; row++)
		{
			for (int col = 0; col < 3; col++)
				pose(row, col) = R0.at<float>(row, col);
			pose(row, 3) = t0.at<float>(row);
		}
		pose = pose.inverse();

		LOG_INFO("PnPL: nbInliers: {}/{} points, {}/{} lines", imagePoints_inliers.size(), imagePoints.size(), imageLines_inliers.size(), imageLines.size());
		return FrameworkReturnCode::_SUCCESS;
	}


	void SolARPoseEstimationPnPL::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams)
	{
		this->m_camDistortion.at<float>(0, 0) = distortionParams(0);
		this->m_camDistortion.at<float>(1, 0) = distortionParams(1);
		this->m_camDistortion.at<float>(2, 0) = distortionParams(2);
		this->m_camDistortion.at<float>(3, 0) = distortionParams(3);
		this->m_camDistortion.at<float>(4, 0) = distortionParams(4);
		this->m_camMatrix.at<float>(0, 0) = intrinsicParams(0, 0);
		this->m_camMatrix.at<float>(0, 1) = intrinsicParams(0, 1);
		this->m_camMatrix.at<float>(0, 2) = intrinsicParams(0, 2);
		this->m_camMatrix.at<float>(1, 0) = intrinsicParams(1, 0);
		this->m_camMatrix.at<float>(1, 1) = intrinsicParams(1, 1);
		this->m_camMatrix.at<float>(1, 2) = intrinsicParams(1, 2);
		this->m_camMatrix.at<float>(2, 0) = intrinsicParams(2, 0);
		this->m_camMatrix.at<float>(2, 1) = intrinsicParams(2, 1);
		this->m_camMatrix.at<float>(2, 2) = intrinsicParams(2, 2);

		cv::invert(m_camMatrix, m_Kinv);
	}

	bool SolARPoseEstimationPnPL::pnpl_main(const std::vector<Point3Df>& pt3d, const std::vector<Point2Df>& pt2d, const std::vector<Edge3Df>& ln3d, const std::vector<Edge2Df>& ln2d, cv::Mat & R0, cv::Mat & t0)
	{
		int Np = pt2d.size();
		int Nl = ln2d.size();

		cv::Mat A(2 * (Np + Nl), 12, CV_32F);

		// Points
		cv::Mat ui(3, 1, CV_32F);
		float u, v, X, Y, Z;
		for (int i = 0; i < Np; i++)
		{
			ui = (cv::Mat_<float>(3, 1) << pt2d[i].getX(), pt2d[i].getY(), 1);
			ui = m_Kinv * ui; ui /= cv::norm(ui);
			u = ui.at<float>(0); v = ui.at<float>(1);
			X = pt3d[i].getX(); Y = pt3d[i].getY(); Z = pt3d[i].getZ();

			// [ r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3]
			A.row(2 * i) = (cv::Mat_<float>(1, 12, CV_32F) << -X, -Y, -Z, 0, 0, 0, u * X, u * Y, u * Z, -1, 0, u);
			A.row(2 * i + 1) = (cv::Mat_<float>(1, 12, CV_32F) << 0, 0, 0, -X, -Y, -Z, v * X, v * Y, v * Z, 0, -1, v);
		}
		// Lines
		cv::Mat s(3, 1, CV_32F), e(3, 1, CV_32F), l(3, 1, CV_32F);
		float l1, l2, l3;
		float Xs, Ys, Zs, Xe, Ye, Ze;
		for (int i = 0; i < Nl; i++)
		{
			s = (cv::Mat_<float>(3, 1, CV_32F) << ln2d[i].p1.getX(), ln2d[i].p1.getY(), 1);
			e = (cv::Mat_<float>(3, 1, CV_32F) << ln2d[i].p2.getX(), ln2d[i].p2.getY(), 1);
			l = s.cross(e); l /= cv::norm(l);
			l1 = l.at<float>(0); l2 = l.at<float>(1); l3 = l.at<float>(2);
			Xs = ln3d[i].p1.getX(); Ys = ln3d[i].p1.getY(); Zs = ln3d[i].p1.getZ();
			Xe = ln3d[i].p2.getX(); Ye = ln3d[i].p2.getY(); Ze = ln3d[i].p2.getZ();

			// [ r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3]
			A.row(2 * (Np + i)) = (cv::Mat_<float>(1, 12, CV_32F) << l1 * Xs, l1 * Ys, l1 * Zs, l2 * Xs, l2 * Ys, l2 * Zs, l3 * Xs, l3 * Ys, l3 * Zs, l1, l2, l3);
			A.row(2 * (Np + i) + 1) = (cv::Mat_<float>(1, 12, CV_32F) << l1 * Xe, l1 * Ye, l1 * Ze, l2 * Xe, l2 * Ye, l2 * Ze, l3 * Xe, l3 * Ye, l3 * Ze, l1, l2, l3);
		}
		// Solve with SVD
		cv::Mat W, U, V, S;
		cv::SVD::compute(A, W, U, V);
		S = V.row(11).t();
		//cv::SVD::solveZ(A, S);
		if (std::isnan(S.at<float>(0)) || S.at<float>(0) == 0)
			return false;
		// Retrieve pose
		cv::Mat rotMat(3, 3, CV_32F);
		for (int row = 0; row < 3; row++)
			for (int col = 0; col < 3; col++)
				rotMat.at<float>(row, col) = S.at<float>(3 * row + col);
		// Normalize rotMat
		cv::Mat Rvec;
		cv::Rodrigues(rotMat, Rvec);
		cv::Rodrigues(Rvec, rotMat);
		R0 = rotMat;
		// Translation vector
		cv::Mat Tvec = (cv::Mat_<float>(3, 1, CV_32F) << S.at<float>(9), S.at<float>(10), S.at<float>(11));
		t0 = Tvec;

		return true;
	}

	/* 
	 * [WIP] OPnPL
	 * Vakhitov, Alexander & Funke, Jan & Moreno-Noguer, Francesc. (2016). Accurate and Linear Time Pose Estimation from Points and Lines.
	 * Adapted from the MATLAB code provided here under MIT License: https://github.com/alexandervakhitov/pnpl
	 */
	bool SolARPoseEstimationPnPL::opnpl_main1(	const std::vector<Point3Df> & pt3d,
												const std::vector<Point2Df> &  pt2d,
												const std::vector<Edge3Df> & ln3d,
												const std::vector<Edge2Df> & ln2d,
												cv::Mat & R0, cv::Mat & t0,
												float error)
	{
		int nbPoints = pt3d.size();
		int nbLines = ln3d.size();
		// Compute average 3D point
		Point3Df center3D;
		for (Point3Df p : pt3d)
			center3D += p;
		for (Edge3Df e : ln3d)
			center3D += e.p1 + e.p2;
		center3D *= 1.0f / float(nbPoints + 2 * nbLines);
		// Translate 3D points wrt centroid
		std::vector<Point3Df> pt3d_c;
		for (int i = 0; i < nbPoints; i++)
			pt3d_c.push_back(pt3d[i] - center3D);

		// Build point matrices Mp & Np: 2*nbPoints x 11 & 2
		cv::Mat Mp = cv::Mat(2 * nbPoints, 11, CV_32F);
		cv::Mat Np = cv::Mat(2 * nbPoints, 2, CV_32F);

		float u, v, X, Y, Z, X_c, Y_c, Z_c;
		cv::Mat u1, um1;
		for (int i = 0; i < nbPoints; i++)
		{
			u1	= (cv::Mat_<float>(3, 1, CV_32F) << pt2d[i].getX(), pt2d[i].getY(), 1.f);
			um1 = m_Kinv * u1;
			um1 /= um1.at<float>(2);
			u	= um1.at<float>(0);		v	= um1.at<float>(1);
			X	= pt3d[i].getX();		Y	= pt3d[i].getY();	Z	= pt3d[i].getZ();
			X_c = pt3d_c[i].getX();		Y_c = pt3d_c[i].getY(); Z_c = pt3d_c[i].getZ();

			Mp.at<float>(2 * i, 0)  = u;
			Mp.at<float>(2 * i, 1)  = u * Z_c - X;
			Mp.at<float>(2 * i, 2)  = 2 * u * Y_c;
			Mp.at<float>(2 * i, 3)  = -2 * (Z + u * X_c);
			Mp.at<float>(2 * i, 4)  = 2 * Y;
			Mp.at<float>(2 * i, 5)  = -(X + u * Z_c);
			Mp.at<float>(2 * i, 6)  = -2 * Y;
			Mp.at<float>(2 * i, 7)  = 2 * (u * X_c - Z);
			Mp.at<float>(2 * i, 8)  = X - u * Z_c;
			Mp.at<float>(2 * i, 9)  = 2 * u * Y_c;
			Mp.at<float>(2 * i, 10) = X + u * Z_c;

			Np.at<float>(2 * i, 0)  = -1;
			Np.at<float>(2 * i, 1)  =  0;

			Mp.at<float>(2 * i + 1, 0)  = v;
			Mp.at<float>(2 * i + 1, 1)  = v * Z_c - Y;
			Mp.at<float>(2 * i + 1, 2)  = 2 * (Z + v * Y_c);
			Mp.at<float>(2 * i + 1, 3)  = -2 * (v * X_c);
			Mp.at<float>(2 * i + 1, 4)  = -2 * X;
			Mp.at<float>(2 * i + 1, 5)  = Y - v * Z_c;
			Mp.at<float>(2 * i + 1, 6)  = -2 * X;
			Mp.at<float>(2 * i + 1, 7)  = 2 * v * X_c;
			Mp.at<float>(2 * i + 1, 8)  = -(Y + v * Z_c);
			Mp.at<float>(2 * i + 1, 9)  = 2 * (v * Y_c - Z);
			Mp.at<float>(2 * i + 1, 10) = Y + v * Z_c;

			Np.at<float>(2 * i + 1, 0)  =  0;
			Np.at<float>(2 * i + 1, 1)  = -1;
		}
		// Build line matrices Ml & Nl: 2*nbLines x 11 & 2
		cv::Mat Ml = cv::Mat(2 * nbLines, 11, CV_32F);
		cv::Mat Nl = cv::Mat(2 * nbLines, 2, CV_32F);

		float l1, l2, l3, Xs, Ys, Zs, Xe, Ye, Ze;
		cv::Mat p1, p2;
		cv::Mat pm1, pm2, l;
		float l_norm;
		for (int i = 0; i < nbLines; i++)
		{
			p1 = (cv::Mat_<float>(3, 1, CV_32F) << ln2d[i].p1.getX(), ln2d[i].p1.getY(), 1.f);
			p2 = (cv::Mat_<float>(3, 1, CV_32F) << ln2d[i].p2.getX(), ln2d[i].p2.getY(), 1.f);
			pm1 = m_Kinv * p1;
			pm2 = m_Kinv * p2;
			pm1 /= pm1.at<float>(2);
			pm2 /= pm2.at<float>(2);

			// Get line coords
			l = pm1.cross(pm2);
			l_norm = sqrt(pow(l.at<float>(0), 2) + pow(l.at<float>(1), 2));
			l /= l_norm;
			l1 = l.at<float>(0);		l2 = l.at<float>(1);		l3 = l.at<float>(2);
			Xs  = ln3d[i].p1.getX();	Ys  = ln3d[i].p1.getY();	Zs  = ln3d[i].p1.getZ();
			Xe  = ln3d[i].p2.getX();	Ye  = ln3d[i].p2.getY();	Ze  = ln3d[i].p2.getZ();
			X_c = center3D.getX();		Y_c = center3D.getY();		Z_c = center3D.getZ();

			Ml.at<float>(i, 0)  =		l3;
			Ml.at<float>(i, 1)  =		l1 * Xs + l2 * Ys + l3 * Zs - l3 * Z_c;
			Ml.at<float>(i, 2)  = 2 * (-l2 * Zs + l3 * Ys - l3 * Y_c );
			Ml.at<float>(i, 3)  = 2 * ( l1 * Zs - l3 * Xs + l3 * X_c );
			Ml.at<float>(i, 4)  = 2 * (-l1 * Ys + l2 * Xs );
			Ml.at<float>(i, 5)  =	    l1 * Xs - l2 * Ys - l3 * Zs + l3 * Z_c;
			Ml.at<float>(i, 6)  = 2 * ( l1 * Ys + l2 * Xs );
			Ml.at<float>(i, 7)  = 2 * ( l1 * Zs + l3 * Xs - l3 * X_c );
			Ml.at<float>(i, 8)  =	   -l1 * Xs + l2 * Ys - l3 * Zs + l3 * Z_c;
			Ml.at<float>(i, 9)  = 2 * ( l2 * Zs + l3 * Ys - l3 * Y_c );
			Ml.at<float>(i, 10) =	   -l1 * Xs - l2 * Ys + l3 * Zs - l3 * Z_c;

			Nl.at<float>(i, 0) = l1;
			Nl.at<float>(i, 1) = l2;

			Ml.at<float>(nbLines + i, 0)  =		  l3;
			Ml.at<float>(nbLines + i, 1)  =		  l1 * Xe + l2 * Ye + l3 * Ze - l3 * Z_c;
			Ml.at<float>(nbLines + i, 2)  = 2 * (-l2 * Ze + l3 * Ye - l3 * Y_c);
			Ml.at<float>(nbLines + i, 3)  = 2 * ( l1 * Ze - l3 * Xe + l3 * X_c);
			Ml.at<float>(nbLines + i, 4)  = 2 * (-l1 * Ye + l2 * Xe);
			Ml.at<float>(nbLines + i, 5)  =		  l1 * Xe - l2 * Ye - l3 * Ze + l3 * Z_c;
			Ml.at<float>(nbLines + i, 6)  = 2 * ( l1 * Ye + l2 * Xe);
			Ml.at<float>(nbLines + i, 7)  = 2 * ( l1 * Ze + l3 * Xe - l3 * X_c);
			Ml.at<float>(nbLines + i, 8)  =		 -l1 * Xe + l2 * Ye - l3 * Ze + l3 * Z_c;
			Ml.at<float>(nbLines + i, 9)  = 2 * ( l2 * Ze + l3 * Ye - l3 * Y_c);
			Ml.at<float>(nbLines + i, 10) =		 -l1 * Xe - l2 * Ye + l3 * Ze - l3 * Z_c;

			Nl.at<float>(nbLines + i, 0) = l1;
			Nl.at<float>(nbLines + i, 1) = l2;
		}
		// Concatenate matrices
		cv::Mat M, N; // 2 * (nbPoints + nbLines) x 11 & 2
		cv::vconcat(std::vector<cv::Mat>{ Mp, Ml }, M);
		cv::vconcat(std::vector<cv::Mat>{ Np, Nl }, N);
		cv::Mat Mt, Nt;
		Mt = M.t(); Nt = N.t();


		cv::Mat wA, uA, vtA, wB, uB, vtB, wC, uC, vtC, wD, uD, vtD;
		cv::Mat SOL;
		cv::SVD::solveZ(M, SOL);
		LOG_INFO("X\n{}", SOL);
		float a2 = SOL.at<float>(1);
		float b2 = SOL.at<float>(5);
		float c2 = SOL.at<float>(8);
		float d2 = SOL.at<float>(10);
		LOG_INFO("a^2:{}, b^2:{}, c^2:{}, d^2:{}", a2, b2, c2, d2);
		float lambda = 1 / (a2 + b2 + c2 + d2);
		LOG_INFO("lambda1:{}", lambda);
		LOG_INFO("a:{}, b:{}, c:{}, d:{}", sqrt(a2 * lambda), sqrt(b2 * lambda), sqrt(c2 * lambda), sqrt(d2 * lambda));
		//LOG_INFO("M:\n{}", M);
		//LOG_INFO("N:\n{}", N);

		// Compute solver parameters
		cv::Mat P, Q, P2;
		P = -(Nt * N).inv() * Nt * M;
		Q = Mt * M + Mt * N * P;

		//LOG_INFO("P:\n{}", P);
		//LOG_INFO("Q:\n{}", Q);
											
		cv::Mat A = (cv::Mat_<float>(1, 24, CV_32F) <<
			4 * Q.at<float>(1, 1),								6 * Q.at<float>(1, 2),								6 * Q.at<float>(1, 3),								6 * Q.at<float>(1, 4),
			4 * Q.at<float>(1, 5)  + 2 * Q.at<float>(2, 2),		4 * Q.at<float>(1, 6)  + 4 * Q.at<float>(2, 3),		4 * Q.at<float>(1, 7) + 4 * Q.at<float>(2, 4),		4 * Q.at<float>(1, 8) + 2 * Q.at<float>(3, 3),
			4 * Q.at<float>(1, 9)  + 4 * Q.at<float>(3, 4),		4 * Q.at<float>(1, 10) + 2 * Q.at<float>(4, 4),		4 * Q.at<float>(0, 1),								2 * Q.at<float>(2, 5),
			2 * Q.at<float>(2, 6)  + 2 * Q.at<float>(3, 5),		2 * Q.at<float>(2, 7)  + 2 * Q.at<float>(4, 5),		2 * Q.at<float>(2, 8) + 2 * Q.at<float>(3, 6),		2 * Q.at<float>(2, 9) + 2 * Q.at<float>(3, 7) + 2 * Q.at<float>(4, 6),
			2 * Q.at<float>(2, 10) + 2 * Q.at<float>(4, 7),		2 * Q.at<float>(0, 2),								2 * Q.at<float>(3, 8),								2 * Q.at<float>(3, 9) + 2 * Q.at<float>(4, 8),
			2 * Q.at<float>(3, 10) + 2 * Q.at<float>(4, 9),		2 * Q.at<float>(0, 3),								2 * Q.at<float>(4, 10),								2 * Q.at<float>(0, 4));

		cv::Mat B = (cv::Mat_<float>(1, 24, CV_32F) <<
			2 * Q.at<float>(1, 2),														4 * Q.at<float>(1, 5)  + 2 * Q.at<float>(2, 2),		2 * Q.at<float>(1, 6) + 2 * Q.at<float>(2, 3),		2 * Q.at<float>(1, 7) + 2 * Q.at<float>(2, 4),
			6 * Q.at<float>(2, 5),														4 * Q.at<float>(2, 6)  + 4 * Q.at<float>(3, 5),		4 * Q.at<float>(2, 7) + 4 * Q.at<float>(4, 5),		2 * Q.at<float>(2, 8) + 2 * Q.at<float>(3, 6),
			2 * Q.at<float>(2, 9)  + 2 * Q.at<float>(3, 7) + 2 * Q.at<float>(4, 6),		2 * Q.at<float>(2, 10) + 2 * Q.at<float>(4, 7),		2 * Q.at<float>(0, 2),								4 * Q.at<float>(5, 5),
			6 * Q.at<float>(5, 6),														6 * Q.at<float>(5, 7),								4 * Q.at<float>(5, 8) + 2 * Q.at<float>(6, 6),		4 * Q.at<float>(5, 9) + 4 * Q.at<float>(6, 7),
			4 * Q.at<float>(5, 10) + 2 * Q.at<float>(7, 7),								4 * Q.at<float>(0, 5),								2 * Q.at<float>(6, 8),								2 * Q.at<float>(6, 9) + 2 * Q.at<float>(7, 8),
			2 * Q.at<float>(6, 10) + 2 * Q.at<float>(7, 9),								2 * Q.at<float>(0, 6),								2 * Q.at<float>(7, 10),								2 * Q.at<float>(0, 7));

		cv::Mat C = (cv::Mat_<float>(1, 24, CV_32F) <<
			2 * Q.at<float>(1, 3),								2 * Q.at<float>(1, 6)  + 2 * Q.at<float>(2, 3),		4 * Q.at<float>(1, 8) + 2 * Q.at<float>(3, 3),								2 * Q.at<float>(1, 9) + 2 * Q.at<float>(3, 4),
			2 * Q.at<float>(2, 6)  + 2 * Q.at<float>(3, 5),		4 * Q.at<float>(2, 8)  + 4 * Q.at<float>(3, 6),		2 * Q.at<float>(2, 9) + 2 * Q.at<float>(3, 7) + 2 * Q.at<float>(4, 6),		6 * Q.at<float>(3, 8),
			4 * Q.at<float>(3, 9)  + 4 * Q.at<float>(4, 8),		2 * Q.at<float>(3, 10) + 2 * Q.at<float>(4, 9),		2 * Q.at<float>(0, 3),														2 * Q.at<float>(5, 6),
			4 * Q.at<float>(5, 8)  + 2 * Q.at<float>(6, 6),		2 * Q.at<float>(5, 9)  + 2 * Q.at<float>(6, 7),		6 * Q.at<float>(6, 8),														4 * Q.at<float>(6, 9) + 4 * Q.at<float>(7, 8),
			2 * Q.at<float>(6, 10) + 2 * Q.at<float>(7, 9),		2 * Q.at<float>(0, 6),								4 * Q.at<float>(8, 8),														6 * Q.at<float>(8, 9),
			4 * Q.at<float>(8, 10) + 2 * Q.at<float>(9, 9),		4 * Q.at<float>(0, 8),								2 * Q.at<float>(9, 10),														2 * Q.at<float>(0, 9));

		cv::Mat D = (cv::Mat_<float>(1, 24, CV_32F) <<
			2 * Q.at<float>(1, 4),								2 * Q.at<float>(1, 7)  + 2 * Q.at<float>(2, 4),								2 * Q.at<float>(1, 9)  + 2 * Q.at<float>(3, 4),		4 * Q.at<float>(1, 10) + 2 * Q.at<float>(4, 4),
			2 * Q.at<float>(2, 7)  + 2 * Q.at<float>(4, 5),		2 * Q.at<float>(2, 9)  + 2 * Q.at<float>(3, 7) + 2 * Q.at<float>(4, 6),		4 * Q.at<float>(2, 10) + 4 * Q.at<float>(4, 7),		2 * Q.at<float>(3, 9)  + 2 * Q.at<float>(4, 8),
			4 * Q.at<float>(3, 10) + 4 * Q.at<float>(4, 9),		6 * Q.at<float>(4, 10),														2 * Q.at<float>(0, 4),								2 * Q.at<float>(5, 7),
			2 * Q.at<float>(5, 9)  + 2 * Q.at<float>(6, 7),		4 * Q.at<float>(5, 10) + 2 * Q.at<float>(7, 7),								2 * Q.at<float>(6, 9)  + 2 * Q.at<float>(7, 8),		4 * Q.at<float>(6, 10) + 4 * Q.at<float>(7, 9),
			6 * Q.at<float>(7, 10),								2 * Q.at<float>(0, 7),														2 * Q.at<float>(8, 9),								4 * Q.at<float>(8, 10) + 2 * Q.at<float>(9, 9),
			6 * Q.at<float>(9, 10),								2 * Q.at<float>(0, 9),														4 * Q.at<float>(10, 10),							4 * Q.at<float>(0, 10));

		//LOG_INFO("A\n{}", A);
		//LOG_INFO("B\n{}", B);
		//LOG_INFO("C\n{}", C);
		//LOG_INFO("D\n{}", D);

		A = A / cv::norm(A);
		B = B / cv::norm(B);
		C = C / cv::norm(C);
		D = D / cv::norm(D);

		//LOG_INFO("Anorm\n{}", A);
		//LOG_INFO("Bnorm\n{}", B);
		//LOG_INFO("Cnorm\n{}", C);
		//LOG_INFO("Dnorm\n{}", D);

		cv::Mat coeffs1, coeffs2, coeffs;
		cv::vconcat(A, B, coeffs1);
		cv::vconcat(C, D, coeffs2);
		cv::vconcat(coeffs1, coeffs2, coeffs);
		cv::Mat sols;

		cv::Mat An(1, 24, CV_32F), Bn(1, 24, CV_32F), Cn(1, 24, CV_32F), Dn(1, 24, CV_32F);
		for (int i = 0; i < 24; i++)
		{
			An.at<float>(i) = A.at<float>(23 - i);
			Bn.at<float>(i) = B.at<float>(23 - i);
			Cn.at<float>(i) = C.at<float>(23 - i);
			Dn.at<float>(i) = D.at<float>(23 - i);
		}

		// Polynomial solver
		cv::Mat XX, YY, ZZ, TT;
		//cv::Mat roots;
		int max_iter = 300;
		cv::solvePoly(A, XX, max_iter);
		cv::solvePoly(B, YY, max_iter);
		cv::solvePoly(C, ZZ, max_iter);
		cv::solvePoly(D, TT, max_iter);
		//cv::SVD::compute(A, wA, uA, vtA);
		//cv::SVD::compute(B, wB, uB, vtB);
		//cv::SVD::compute(C, wC, uC, vtC);
		//cv::SVD::compute(D, wD, uD, vtD);
		//cv::Mat wA, uA, vtA, wB, uB, vtB, wC, uC, vtC, wD, uD, vtD;
		//cv::SVD::solveZ(M, SOL);
		//LOG_INFO("A\n{}, X\n{}", M, SOL);
		//cv::Mat SOL = vtA.row(3).t();
		//LOG_DEBUG("X\n{}", SOL);
		//LOG_DEBUG("a:{}, b:{}, c:{}, d:{}", SOL.at<float>(10), SOL.at<float>(17), SOL.at<float>(21), SOL.at<float>(23));

		//LOG_DEBUG("A\n{}, w\n{}, u\n{}, vt\n{}", A, wA, uA, vtA);
		//LOG_DEBUG("B\n{}, w\n{}, u\n{}, vt\n{}", B, wB, uB, vtB);
		//LOG_DEBUG("C\n{}, w\n{}, u\n{}, vt\n{}", C, wC, uC, vtC);
		//LOG_DEBUG("D\n{}, w\n{}, u\n{}, vt\n{}", D, wD, uD, vtD);

		//LOG_DEBUG("XX: {}", XX);
		//LOG_DEBUG("YY: {}", YY);
		//LOG_DEBUG("ZZ: {}", ZZ);
		//LOG_DEBUG("TT: {}", TT);

		// Remove repetitive solutions (equals or opposites) and keep only real
		std::vector<float> XX_f, YY_f, ZZ_f, TT_f;
		float xi, yi, zi, ti;
		float xj, yj, zj, tj;
		cv::Mat tmpi, tmpj;
		bool isRepeated;
		for (int i = 0; i < XX.rows; i++)
		{
			isRepeated = false;
			for (int j = i + 1; j < XX.rows; j++)
			{
				xi = XX.at<float>(i, 0); xj = XX.at<float>(j, 0);
				yi = YY.at<float>(i, 0); yj = YY.at<float>(j, 0);
				zi = ZZ.at<float>(i, 0); zj = ZZ.at<float>(j, 0);
				ti = TT.at<float>(i, 0); tj = TT.at<float>(j, 0);

				tmpi = (cv::Mat_<float>(1, 4, CV_32F) << xi, yi, zi, ti);
				tmpj = (cv::Mat_<float>(1, 4, CV_32F) << xj, yj, zj, tj);

				if (cv::norm(tmpi - tmpj) < 1e-4 || cv::norm(tmpi + tmpj) < 1e-4)
					isRepeated = true;
			}
			if (!isRepeated)
			{
				if (abs(XX.at<float>(i, 1)) <= 1e-10)
					XX_f.push_back(XX.at<float>(i, 0));
				if (abs(YY.at<float>(i, 1)) <= 1e-10)
					YY_f.push_back(YY.at<float>(i, 0));
				if (abs(ZZ.at<float>(i, 1)) <= 1e-10)
					ZZ_f.push_back(ZZ.at<float>(i, 0));
				if (abs(TT.at<float>(i, 1)) <= 1e-10)
				TT_f.push_back(TT.at<float>(i, 0));
			}
		}

		for (auto sol : XX_f)
			LOG_INFO("XX_f: {}", sol);
		for (auto sol : YY_f)
			LOG_INFO("YY_f: {}", sol);
		for (auto sol : ZZ_f)
			LOG_INFO("ZZ_f: {}", sol);
		for (auto sol : TT_f)
			LOG_INFO("TT_f: {}", sol);

		// TODO refine solutions

		// Retrieve pose [R t]
		error = FLT_MAX;
		//for (int i = 0; i < XX.rows; i++)
		for (auto a : XX_f)
		for (auto b : YY_f)
		for (auto c : ZZ_f)
		for (auto d : TT_f)
		{
			//a = XX.at<float>(i, 0);
			//b = YY.at<float>(i, 0);
			//c = ZZ.at<float>(i, 0);
			//d = TT.at<float>(i, 0);

			//LOG_INFO("a, b, c, d: [{}; {}; {}; {}]", a, b, c, d);

			float lambda1 = 1 / (a * a + b * b + c * c + d * d);
			if (lambda1 > 1e10)
				continue;

			cv::Mat vec = (cv::Mat_<float>(1, 11, CV_32F) << 1, a*a, a*b, a*c, a*d, b*b, b*c, b*d, c*c, c*d, d*d);

			a *= sqrt(lambda1);
			b *= sqrt(lambda1);
			c *= sqrt(lambda1);
			d *= sqrt(lambda1);


			cv::Mat R = (cv::Mat_<float>(3, 3, CV_32F) <<
				a * a + b * b - c * c - d * d, 2 * b * c - 2 * a * d, 2 * b * d + 2 * a * c,
				2 * b * c + 2 * a * d, a * a - b * b + c * c - d * d, 2 * c * d - 2 * a * b,
				2 * b * d - 2 * a * c, 2 * c * d + 2 * a * b, a * a - b * b - c * c + d * d);


			cv::Mat t12 = P * vec.t() * lambda1;
			cv::Mat t3 = R.row(2) * c;
			cv::Mat c = (cv::Mat_<float>(3, 1, CV_32F) << center3D.getX(), center3D.getY(), center3D.getZ());
			cv::Mat t = (cv::Mat_<float>(3, 1, CV_32F) << t12.at<float>(0, 0), t12.at<float>(1, 0), lambda1 - t3.at<float>(0, 0));

			//if (i == 0)
			//{
			//	std::vector<cv::Mat> reprojected2d_points;
			//	cv::Mat pt3D_mat;
			//	cv::Mat pt2d_proj;
			//	for (int j = 0; j < 1; j++)
			//	{
			//		pt3D_mat = (cv::Mat_<float>(3, 1, CV_32F) << pt3d[j].getX(), pt3d[j].getY(), pt3d[j].getZ());
			//		pt2d_proj = m_camMatrix * (R * pt3D_mat + t);
			//		reprojected2d_points.push_back(pt2d_proj);
			//		//LOG_INFO("pt2d: {}\nreproj: {}", pt2d[j], pt2d_proj / pt2d_proj.at<float>(2, 0));
			//	}
			//}

			cv::Mat err = vec * Q * vec.t();
			// Compare and keep best solution
			if (err.at<float>(0, 0) < error)
			{
				error = err.at<float>(0, 0);
				R0 = R;
				t0 = t;
			}
		}
		//LOG_INFO("R0, t0: {}, {}", R0, t0);
		LOG_INFO("error: {}", error);
		return true;
	}

	float SolARPoseEstimationPnPL::getPointReprojError(const Point2Df pt2D, const Point3Df pt3D, const cv::Mat& R, const cv::Mat& t)
	{
		cv::Mat U = (cv::Mat_<float>(3, 1, CV_32F) << pt3D.getX(), pt3D.getY(), pt3D.getZ());
		cv::Mat u = m_camMatrix * (R * U + t);
		Point2Df pt2D_reproj(u.at<float>(0) / u.at<float>(2), u.at<float>(1) / u.at<float>(2));
		return (pt2D - pt2D_reproj).magnitude();
	}

	float SolARPoseEstimationPnPL::getLineReprojError(const Edge2Df ln2D, const Edge3Df ln3D, const cv::Mat& R, const cv::Mat& t)
	{
		cv::Mat Xs = (cv::Mat_<float>(3, 1, CV_32F) << ln3D.p1.getX(), ln3D.p1.getY(), ln3D.p1.getY());
		cv::Mat Xe = (cv::Mat_<float>(3, 1, CV_32F) << ln3D.p2.getX(), ln3D.p2.getY(), ln3D.p2.getY());
		cv::Mat xs = m_camMatrix * (R * Xs + t);
		cv::Mat xe = m_camMatrix * (R * Xe + t);
		cv::Mat u1 = (cv::Mat_<float>(3, 1, CV_32F) << ln2D.p1.getX(), ln2D.p1.getY(), 1.f);
		cv::Mat u2 = (cv::Mat_<float>(3, 1, CV_32F) << ln2D.p2.getX(), ln2D.p2.getY(), 1.f);
		cv::Mat l1 = u1.cross(u2); l1 /= cv::norm(l1);
		cv::Mat l2 = xs.cross(xe); l1 /= cv::norm(l1);
		return cv::norm(l1 - l2);
	}

	cv::Point3f SolARPoseEstimationPnPL::normalizedLineCoeff(const Edge2Df line)
	{
		cv::Point3f start = cv::Point3f(line.p1(0), line.p1(1), 1);
		cv::Point3f end = cv::Point3f(line.p2(0), line.p2(1), 1);

		cv::Point3f lineCoeff = start.cross(end);
		float magnitude = sqrt(pow(lineCoeff.x, 2) + pow(lineCoeff.y, 2) + pow(lineCoeff.z, 2));
			
		return cv::Point3f(lineCoeff.x / magnitude, lineCoeff.y / magnitude, lineCoeff.z / magnitude);
	}

	float SolARPoseEstimationPnPL::algebraicPointLineError(const Point3Df P, const cv::Point3f line2DCoeffs, const cv::Matx34f Pose)
	{
		cv::Point3f p(P(0), P(1), P(2));
		// Project 3D point
		cv::Mat_<float> KPose = m_camMatrix * cv::Mat(Pose);
		cv::Mat_<float> proj_2d = KPose * cv::Mat_<float>(p);
		cv::Point3f proj(proj_2d(0), proj_2d(1), proj_2d(2));
		// Compute distance between line and projected point
		cv::Mat_<float> error = cv::Mat_<float>(line2DCoeffs) * cv::Mat_<float>(proj);
		LOG_DEBUG("error size", error.size());
		return error(0);
	}

	float SolARPoseEstimationPnPL::algebraicLineSegmentError(const Edge3Df line3D, const Edge2Df line2D, const cv::Matx34f Pose)
	{
		Point3Df P = line3D.p1;
		Point3Df Q = line3D.p2;
		cv::Point3f lineCoeffs = normalizedLineCoeff(line2D);

		float errP = algebraicPointLineError(P, lineCoeffs, Pose);
		float errQ = algebraicPointLineError(Q, lineCoeffs, Pose);

		return pow(errP, 2) + pow(errQ, 2);
	}

	float SolARPoseEstimationPnPL::lineSegmentError(const std::vector<Edge3Df> lines3D, const std::vector<Edge2Df> lines2D, const Transform3Df pose)
	{
		if (lines3D.size() != lines2D.size())
		{
			LOG_WARNING("2D-3D correspondances don't match in size!");
			return -1;
		}

		cv::Matx34f Pose(pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3),
						 pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3),
						 pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3));

		float error = 0.0f;
		for (unsigned i = 0; i < lines3D.size(); i++)
		{
			error += algebraicLineSegmentError(lines3D[i], lines2D[i], Pose);
		}
		return error;
	}

}
}
}


