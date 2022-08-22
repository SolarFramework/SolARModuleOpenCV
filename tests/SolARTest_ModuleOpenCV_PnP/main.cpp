#include <iostream>
#include <fstream>

#include <string>
#include <vector>


//#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"

#include <stdlib.h>
#include <stdio.h>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <random>

#include "random_generators.hpp"
#include "experiment_helpers.hpp"
#include "time_measurement.hpp"

#include <boost/log/core.hpp>

using namespace Eigen;
using namespace SolAR::PnPTest;

#include "core/Log.h"

// ADD COMPONENTS HEADERS HERE

#include "xpcf/xpcf.h"

#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "api/solver/pose/I3DTransformSACFinderFrom2D3D.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
namespace xpcf  = org::bcom::xpcf;

void help()
{
    std::cout << "\n\n";
    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
    std::cout << "Something went wrong with input files \n";
    std::cout << "please refer to README.adoc in the project directory \n";
    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n";
    exit(-1);
}

int main()
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    //initialize random seed
    initializeRandomSeed();

    try {

        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if(xpcfComponentManager->load("SolARTest_ModuleOpenCV_PnP_conf.xml")!=org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file SolARTest_ModuleOpenCV_PnP_conf.xml")
            return -1;
        }

        CamCalibration  intrinsicParams;
        //set to identity Matrix
        intrinsicParams(0,0) = 1; intrinsicParams(0,1) = 0; intrinsicParams(0,2) = 0;
        intrinsicParams(1,0) = 0; intrinsicParams(1,1) = 1; intrinsicParams(1,2) = 0;
        intrinsicParams(2,0) = 0; intrinsicParams(2,1) = 0; intrinsicParams(2,2) = 1;

        CamDistortion   distortionParams;
        distortionParams(0,0) =0;
        distortionParams(1,0) =0;
        distortionParams(2,0) =0;
        distortionParams(3,0) =0;
        distortionParams(4,0) =0;

		CameraParameters camParams;
		camParams.intrinsic = intrinsicParams;
		camParams.distortion = distortionParams;


        auto poseEstimation_iterative = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("ITERATIVE");
        auto poseEstimation_epnp = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("EPNP");
        auto poseEstimation_dls = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("DLS");
        auto poseEstimation_upnp = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("UPNP");
        auto poseEstimation_ippe = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("IPPE");

        auto poseEstimation_usac = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("USAC");
        auto poseEstimation_usac_parralel = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("USAC PARALLEL");
        auto poseEstimation_usac_fm_8pts = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("USAC FM 8PTS");
        auto poseEstimation_usac_fast = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("USAC FAST");
        auto poseEstimation_usac_accurate = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("USAC ACCURATE");
        auto poseEstimation_usac_prosac = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("USAC PROSAC");
        auto poseEstimation_usac_magsac = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>("USAC MAGSAC");

		// generate a random pose
		Transform3Df poseGT;
		Vector3f position = generateRandomTranslation(2.0);
		Eigen::Matrix3f rotation = generateRandomRotation(0.5);
		poseGT.translation() = position;
		poseGT.linear() = rotation;
		Transform3Df poseGTInv = poseGT.inverse();

		std::cout << "Ground truth pose: \n" << poseGT.matrix() << std::endl;

        //synthetize 2d points and 3d points to test the components.
        std::vector<Point2Df>  imagePoints;
        std::vector<Point3Df>  worldPoints;
		
		// Nb of correspondences
        unsigned int nbPoints = 2000;

        imagePoints.resize(nbPoints);
        worldPoints.resize(nbPoints);

        for(unsigned int kc =0; kc < nbPoints; kc++){
            Eigen::Vector3f  current_random_point = poseGT * generateRandomPoint();
            //project the point into the viewpoint frame
			Eigen::Vector3f pt2D = poseGTInv * current_random_point;

            imagePoints[kc] = Point2Df(pt2D(0) / pt2D(2), pt2D(1) / pt2D(2));
            worldPoints[kc] = Point3Df(current_random_point(0,0),current_random_point(1,0),current_random_point(2,0));
        }

		// inject outliers
		float outlierFraction = 0.3f;
		uint32_t nbOutliers = (uint32_t)(nbPoints * outlierFraction);
		std::vector<bool> checkInOut(nbPoints, true);
		std::vector<uint32_t> inliersGT;
		for (int i = 0; i < nbOutliers; ++i)
			checkInOut[i] = false;
		auto rng = std::default_random_engine{};
		std::shuffle(checkInOut.begin(), checkInOut.end(), rng);
		for (int i = 0; i < nbPoints; ++i)
			if (!checkInOut[i])
				imagePoints[i] += Point2Df(10, 10);
			else
				inliersGT.push_back(i);

		// Verify output of pnp
		auto verifyPnP = [](const Transform3Df& poseGT, 
			const std::vector<uint32_t>& inliersGT,
			const Transform3Df& pose,
			const std::vector<uint32_t>& inliers) 
		{
			if (!(poseGT.matrix() - pose.matrix()).isZero(1e-3f))
				return false;
			if (inliersGT.size() != inliers.size())
				return false;
			for (int i = 0; i < inliers.size(); ++i)
				if (inliers[i] != inliersGT[i])
					return false;
			return true;
		};

         Transform3Df pose_iterative;
         Transform3Df pose_epnp;
         Transform3Df pose_dls;
         Transform3Df pose_upnp;
         Transform3Df pose_usac;
         Transform3Df pose_usac_parralel;
         Transform3Df pose_usac_fm_8pts;
         Transform3Df pose_usac_fast;
         Transform3Df pose_usac_accurate;
         Transform3Df pose_usac_prosac;
         Transform3Df pose_usac_magsac;

         std::vector<uint32_t> inlier_iterative;
         std::vector<uint32_t> inlier_epnp;
         std::vector<uint32_t> inlier_dls;
         std::vector<uint32_t> inlier_upnp;
         std::vector<uint32_t> inlier_usac;
         std::vector<uint32_t> inlier_usac_parralel;
         std::vector<uint32_t> inlier_usac_fm_8pts;
         std::vector<uint32_t> inlier_usac_fast;
         std::vector<uint32_t> inlier_usac_accurate;
         std::vector<uint32_t> inlier_usac_prosac;
         std::vector<uint32_t> inlier_usac_magsac;

         //timer
         struct timeval tic;
         struct timeval toc;
         size_t iterations = 50;

         std::cout<< "********************* pose_iterative **************************"<<std::endl;
         gettimeofday(&tic, 0);
		 poseEstimation_iterative->estimate(imagePoints, worldPoints, camParams, inlier_iterative, pose_iterative);			 	          
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;
		 std::cout << pose_iterative.matrix() << std::endl;
		 if (verifyPnP(poseGT, inliersGT, pose_iterative, inlier_iterative))
			 std::cout << "====> OK\n\n";
		 else
			 std::cout << "====> NOK\n\n";

         std::cout<< "********************* pose_epnp **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_epnp->estimate(imagePoints, worldPoints, camParams, inlier_epnp, pose_epnp);         
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;
		 std::cout << pose_epnp.matrix() << std::endl;
		 if (verifyPnP(poseGT, inliersGT, pose_epnp, inlier_epnp))
			 std::cout << "====> OK\n\n";
		 else
			 std::cout << "====> NOK\n\n";

         std::cout<< "********************* pose_dls **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_dls->estimate(imagePoints, worldPoints, camParams, inlier_dls, pose_dls);         
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;
		 std::cout << pose_dls.matrix() << std::endl;
		 if (verifyPnP(poseGT, inliersGT, pose_dls, inlier_dls))
			 std::cout << "====> OK\n\n";
		 else
			 std::cout << "====> NOK\n\n";

         std::cout<< "********************* pose_upnp **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_upnp->estimate(imagePoints, worldPoints, camParams, inlier_upnp, pose_upnp);         
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;
		 std::cout << pose_upnp.matrix() << std::endl;
		 if (verifyPnP(poseGT, inliersGT, pose_upnp, inlier_upnp))
			 std::cout << "====> OK\n\n";
		 else
			 std::cout << "====> NOK\n\n";

         std::cout<< "********************* pose_usac **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_usac->estimate(imagePoints, worldPoints, camParams, inlier_usac, pose_usac);         
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;
		 std::cout << pose_usac.matrix() << std::endl;
		 if (verifyPnP(poseGT, inliersGT, pose_usac, inlier_usac))
			 std::cout << "====> OK\n\n";
		 else
			 std::cout << "====> NOK\n\n";

         std::cout<< "********************* pose_usac_fm_8pts **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_usac_fm_8pts->estimate(imagePoints, worldPoints, camParams, inlier_usac_fm_8pts, pose_usac_fm_8pts);         
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;
		 std::cout << pose_usac_fm_8pts.matrix() << std::endl;
		 if (verifyPnP(poseGT, inliersGT, pose_usac_fm_8pts, inlier_usac_fm_8pts))
			 std::cout << "====> OK\n\n";
		 else
			 std::cout << "====> NOK\n\n";

         std::cout<< "********************* pose_usac_fast **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_usac_fast->estimate(imagePoints, worldPoints, camParams, inlier_usac_fast, pose_usac_fast);         
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;
		 std::cout << pose_usac_fast.matrix() << std::endl;
		 if (verifyPnP(poseGT, inliersGT, pose_usac_fast, inlier_usac_fast))
			 std::cout << "====> OK\n\n";
		 else
			 std::cout << "====> NOK\n\n";

         std::cout<< "********************* pose_usac_accurate **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_usac_accurate->estimate(imagePoints, worldPoints, camParams, inlier_usac_accurate, pose_usac_accurate);         
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;
		 std::cout << pose_usac_accurate.matrix() << std::endl;
		 if (verifyPnP(poseGT, inliersGT, pose_usac_accurate, inlier_usac_accurate))
			 std::cout << "====> OK\n\n";
		 else
			 std::cout << "====> NOK\n\n";

         std::cout<< "********************* pose_usac_prosac **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_usac_prosac->estimate(imagePoints, worldPoints, camParams, inlier_usac_prosac, pose_usac_prosac);         
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;
		 std::cout << pose_usac_prosac.matrix() << std::endl;
		 if (verifyPnP(poseGT, inliersGT, pose_usac_prosac, inlier_usac_prosac))
			 std::cout << "====> OK\n\n";
		 else
			 std::cout << "====> NOK\n\n";

         std::cout<< "********************* pose_usac_magsac **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_usac_magsac->estimate(imagePoints, worldPoints, camParams, inlier_usac_magsac, pose_usac_magsac);         
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;
		 std::cout << pose_usac_magsac.matrix() << std::endl;
		 if (verifyPnP(poseGT, inliersGT, pose_usac_magsac, inlier_usac_magsac))
			 std::cout << "====> OK\n\n";
		 else
			 std::cout << "====> NOK\n\n";
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catched: {}", e.what());
        return -1;
    }

    return 0;
}
