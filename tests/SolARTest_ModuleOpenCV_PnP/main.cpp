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

    //set experiment parameters
    double noise = 0.0;
    double outlierFraction = 0.0;
    size_t numberPoints = 100;

    //create a fake central camera
    std::vector<Vector3d, Maths::aligned_allocator<Vector3d> > camOffsets;
    std::vector<Rotationd, Maths::aligned_allocator<Rotationd>> camRotations;
    generateCentralCameraSystem(camOffsets, camRotations);

    //create a random viewpoint pose
    Vector3d position = generateRandomTranslation(2.0);
    Rotationd rotation = generateRandomRotation(0.5);

    //derive correspondences based on random point-cloud
    std::vector<Vector3d, Maths::aligned_allocator<Vector3d>> bearingVectors;
    std::vector<Vector3d, Maths::aligned_allocator<Vector3d> > points;
    std::vector<int> camCorrespondences; //unused in the central case!
    Eigen::MatrixXd gt(3, numberPoints);
    generateRandom2D3DCorrespondences(
        position, rotation, camOffsets, camRotations, numberPoints, noise, outlierFraction,
        bearingVectors, points, camCorrespondences, gt);

    //print the experiment characteristics
    printExperimentCharacteristics(
        position, rotation, noise, outlierFraction);

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


        // initialize pose estimation components with camera paremeters
        poseEstimation_iterative->setCameraParameters(intrinsicParams, distortionParams);
        poseEstimation_epnp->setCameraParameters(intrinsicParams, distortionParams);
        poseEstimation_dls->setCameraParameters(intrinsicParams, distortionParams);
        poseEstimation_upnp->setCameraParameters(intrinsicParams, distortionParams);
        poseEstimation_ippe->setCameraParameters(intrinsicParams, distortionParams);
        poseEstimation_usac->setCameraParameters(intrinsicParams, distortionParams);
        poseEstimation_usac_parralel->setCameraParameters(intrinsicParams, distortionParams);
        poseEstimation_usac_fm_8pts->setCameraParameters(intrinsicParams, distortionParams);
        poseEstimation_usac_fast->setCameraParameters(intrinsicParams, distortionParams);
        poseEstimation_usac_accurate->setCameraParameters(intrinsicParams, distortionParams);
        poseEstimation_usac_prosac->setCameraParameters(intrinsicParams, distortionParams);
        poseEstimation_usac_magsac->setCameraParameters(intrinsicParams, distortionParams);


         //synthetize 2d points and 3d points to test the components.
         std::vector<Point2Df>  imagePoints;
         std::vector<Point3Df>  worldPoints;

         unsigned int Npoints = 1024;

         imagePoints.resize(Npoints);
         worldPoints.resize(Npoints);

         for(unsigned int kc =0; kc < Npoints; kc++){

             double minDepth = 4;
             double maxDepth = 8;

             Eigen::Vector3d  current_random_point = generateRandomPoint( maxDepth, minDepth );

             //get the camera transformation
             Vector3d camOffset = camOffsets[0];
             Rotationd camRotation = camRotations[0];

             //project the point into the viewpoint frame
             Vector3d bodyPoint = rotation.transpose()*(current_random_point - position);

             imagePoints[kc] = Point2Df(bodyPoint(0,0), bodyPoint(1,0)) ;
             worldPoints[kc] = Point3Df(current_random_point(0,0),current_random_point(1,0),current_random_point(2,0));
         }

         poseEstimation_iterative->setCameraParameters(intrinsicParams, distortionParams);
         poseEstimation_epnp->setCameraParameters(intrinsicParams, distortionParams);
         poseEstimation_dls->setCameraParameters(intrinsicParams, distortionParams);
         poseEstimation_upnp->setCameraParameters(intrinsicParams, distortionParams);
         poseEstimation_ippe->setCameraParameters(intrinsicParams, distortionParams);
         poseEstimation_usac->setCameraParameters(intrinsicParams, distortionParams);
         poseEstimation_usac_parralel->setCameraParameters(intrinsicParams, distortionParams);
         poseEstimation_usac_fm_8pts->setCameraParameters(intrinsicParams, distortionParams);
         poseEstimation_usac_fast->setCameraParameters(intrinsicParams, distortionParams);
         poseEstimation_usac_accurate->setCameraParameters(intrinsicParams, distortionParams);
         poseEstimation_usac_prosac->setCameraParameters(intrinsicParams, distortionParams);
         poseEstimation_usac_magsac->setCameraParameters(intrinsicParams, distortionParams);


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
         poseEstimation_iterative->estimate(imagePoints, worldPoints, inlier_iterative, pose_iterative);
         std::cout<<pose_iterative.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_epnp **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_epnp->estimate(imagePoints, worldPoints, inlier_epnp, pose_epnp);
         std::cout<<pose_epnp.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_dls **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_dls->estimate(imagePoints, worldPoints, inlier_dls, pose_dls);
         std::cout<<pose_dls.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_upnp **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_upnp->estimate(imagePoints, worldPoints, inlier_upnp, pose_upnp);
         std::cout<<pose_upnp.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_usac **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_usac->estimate(imagePoints, worldPoints, inlier_usac, pose_usac);
         std::cout<<pose_usac.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_usac_fm_8pts **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_usac_fm_8pts->estimate(imagePoints, worldPoints, inlier_usac_fm_8pts, pose_usac_fm_8pts);
         std::cout<<pose_usac_fm_8pts.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_usac_fast **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_usac_fast->estimate(imagePoints, worldPoints, inlier_usac_fast, pose_usac_fast);
         std::cout<<pose_usac_fast.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_usac_accurate **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_usac_accurate->estimate(imagePoints, worldPoints, inlier_usac_accurate, pose_usac_accurate);
         std::cout<<pose_usac_accurate.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_usac_prosac **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_usac_prosac->estimate(imagePoints, worldPoints, inlier_usac_prosac, pose_usac_prosac);
         std::cout<<pose_usac_prosac.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;

         std::cout<< "********************* pose_usac_magsac **************************"<<std::endl;
         gettimeofday(&tic, 0);
         poseEstimation_usac_magsac->estimate(imagePoints, worldPoints, inlier_usac_magsac, pose_usac_magsac);
         std::cout<<pose_usac_magsac.matrix()<< std::endl;
         gettimeofday(&toc, 0);
         std::cout<<"Computed in "<< TIMETODOUBLE(timeval_minus(toc, tic)) <<"s"<<std::endl;
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catched: {}", e.what());
        return -1;
    }

    return 0;
}
