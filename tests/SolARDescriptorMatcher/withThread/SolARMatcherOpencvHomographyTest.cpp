#include "SolARCameraOpencv.h"
//#include "SolARPoseEstimatorOpencv.h"
#include "SolARMarker2DNaturalImageOpencv.h"
#include "SolARKeypointDetectorOpencv.h"
#include "SolARDescriptorMatcherKNNOpencv.h"
#include "SolARHomographyFinderOpencv.h"
#include "SolARPoseEstimationOpencv.h"
#include "SolARImageViewerOpencv.h"
#include "SolARDescriptorsExtractorSIFTOpencv.h"

#include "SolARCameraTask.h"
#include "SolARKeypointDetectorTask.h"
#include "SolARDescriptorsExtractorTask.h"
#include "SolARDescriptorMatcherTask.h"
#include "SolARPoseHomographyTask.h"
#include "SolARKeypointSplitterTask.h"
#include "boost/log/core.hpp"
#include "SharedBuffer.hpp"

#include "SolARDescriptorBuffer.h"

using namespace SolAR;

namespace xpcf  = org::bcom::xpcf;

#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds



// initialisation
/*int initCamera(sptrnms::shared_ptr<ISolARCamera>& camera,unsigned int device_camera_id, char*  videoFile=NULL){

    boost::uuids::string_generator gen;
    if(camera.get())
        return 0;

    xpcf::ComponentFactory::createComponent<SolARCameraOpencv>(gen(ISolARCamera::UUID),camera);

    if(videoFile==NULL)
        camera->setParameters(640,480,device_camera_id); // w,h,cameraId
    else
        camera->setParameters(640,480,videoFile); // w,h,cameraId

    SolARMatrix33d K;
    SolARMatrix33f intrinsic_param;

    //camera standard

    K(0, 0) = 768.006;   K(0, 1) = 0.0;          K(0, 2) = 401.254;
    K(1, 0) = 0.0;       K(1, 1) = 770.007   ;   K(1, 2) = 305.574;
    K(2, 0) = 0.0;       K(2, 1) = 0.0;          K(2, 2) = 1.0;


    intrinsic_param(0, 0) = 768.006;   intrinsic_param(0, 1) = 0.0;          intrinsic_param(0, 2) = 401.254;
    intrinsic_param(1, 0) = 0.0;       intrinsic_param(1, 1) = 770.007   ;   intrinsic_param(1, 2) = 305.574;
    intrinsic_param(2, 0) = 0.0;       intrinsic_param(2, 1) = 0.0;          intrinsic_param(2, 2) = 1.0;

    camera->setDistorsionParameters(SolARVector4f( 0.0f,0.0f,0.0f,0.0f));

    if ( device_camera_id ==1 ){

        //camera wide angle
        K(0, 0) = 288.48;   K(0, 1) = 0.0;         K(0, 2) = 323.0;
        K(1, 0) = 0.0;      K(1, 1) =288.48   ;    K(1, 2) = 244.86;
        K(2, 0) = 0.0;      K(2, 1) = 0.0;         K(2, 2) = 1.0;

        intrinsic_param(0, 0) = 288.48;   intrinsic_param(0, 1) = 0.0;        intrinsic_param(0, 2) = 323.0;
        intrinsic_param(1, 0) = 0.0;      intrinsic_param(1, 1) =288.48   ;   intrinsic_param(1, 2) = 244.86;
        intrinsic_param(2, 0) = 0.0;      intrinsic_param(2, 1) = 0.0;        intrinsic_param(2, 2) = 1.0;
        camera->setDistorsionParameters(SolARVector4f( 0.0722, -0.216, -0.00133, 0.0003546));
    }

     camera->setIntrinsicParameters(intrinsic_param);

    return 0;
}

*/

int run3(int argc,char* argv[]){

    boost::uuids::string_generator gen;
    boost::log::core::get()->set_logging_enabled(false);


    unsigned int camera_id =0;
    SRef<ISolARCamera> camera;
    xpcf::ComponentFactory::createComponent<SolARCameraOpencv>(gen(ISolARCamera::UUID),camera);

    camera->loadCameraParameters(argv[2]);

    if(argc==4)
        camera->start(argv[3]);
    else
        camera->start(camera_id);



    SRef<ISolARMarker2DNaturalImage> naturalImageMarker;
    xpcf::ComponentFactory::createComponent<SolARMarker2DNaturalImageOpencv>(gen(ISolARMarker2DNaturalImage::UUID ),naturalImageMarker);
    SRef<ISolARKeypointDetector> keypointDetector;
    xpcf::ComponentFactory::createComponent<SolARKeypointDetectorOpencv>(gen(ISolARKeypointDetector::UUID),keypointDetector);
    keypointDetector->setType(SolARKeypointDetectorType::SIFT); // choose SIFT-based keypoint detection

    SRef<ISolARDescriptorsExtractor> descriptorsExtractor0;
    xpcf::ComponentFactory::createComponent<SolARDescriptorsExtractorSIFTOpencv>(gen(ISolARDescriptorsExtractor::UUID),descriptorsExtractor0);

    SRef<ISolARDescriptorsExtractor> descriptorsExtractor1;
    xpcf::ComponentFactory::createComponent<SolARDescriptorsExtractorSIFTOpencv>(gen(ISolARDescriptorsExtractor::UUID),descriptorsExtractor1);

    SRef<SolARDescriptorBuffer> descriptors0;
    SRef<SolARDescriptorBuffer> descriptors1;
    SRef<ISolARDescriptorMatcher>  matcher;
    xpcf::ComponentFactory::createComponent<SolARDescriptorMatcherKNNOpencv>(gen(ISolARDescriptorMatcher::UUID),matcher);

    SRef<ISolARPoseHomography> homographyFinder;
    xpcf::ComponentFactory::createComponent<SolARHomographyFinderOpencv>(gen(ISolARPoseHomography::UUID),homographyFinder);

    // reference image

    SRef<SolARImage> refImage;
    std::vector< SRef<SolARKeypoint> > refKeypoints;

    SRef<SolAR::SolARDescriptorBuffer>  refDescriptors;


    //sptrnms::shared_ptr<ISolARImageLoader> imageLoader;

    //xpcf::ComponentFactory::createComponent<SolARImageLoaderOpencv>(gen(ISolARImageLoader::UUID ),imageLoader);
   // xpcf::ComponentFactory::createComponent<SolARDescriptorOpencv>(gen(ISolARDescriptor::UUID),refDescriptors);

    //imageLoader->loadImage(argv[1],refImage);
    //keypointDetector->detect(refImage,refKeypoints);
    naturalImageMarker->loadMarker(argv[1]);
    naturalImageMarker->getImage(refImage);
    keypointDetector->detect(refImage,refKeypoints);

    std::cout << "nb of keyPoints = " << refKeypoints.size() <<"\n" << std::flush;

    descriptorsExtractor0->compute(refImage,refKeypoints,refDescriptors);

    homographyFinder->setRefDimensions(refImage->getWidth(),refImage->getHeight());
    homographyFinder->setCameraParameters(camera->getIntrinsicsParameters(),camera->getDistorsionParameters());
    homographyFinder->setReferenceKeypoints(refKeypoints);


    // display stuff
    SRef<SolARImage> outputImage;
    SRef<ISolARImageViewer> imageViewer ;
    xpcf::ComponentFactory::createComponent<SolARImageViewerOpencv>(gen(ISolARImageViewer::UUID ),imageViewer);



    std::cout << "process is started \n" << std::endl;

    SharedBuffer<SRef<SolARImage>> outputImageQueue(1);
    SolARCameraTask cameraTask(camera,outputImageQueue);

    SharedFifo<SRef<SolARImage>   > ImageQueueToDescriptor;
    SharedFifo<std::vector< sptrnms::shared_ptr<SolARKeypoint> >  > keypointQueueToDescriptor;
    SharedBuffer<SRef<SolARImage>   > ImageQueueToDisplay(1);
    SharedFifo<std::vector< sptrnms::shared_ptr<SolARKeypoint> >  > keypointQueueToPose;

    SolARKeypointDetectorTask keypointDetectorTask(keypointDetector,
                                                   outputImageQueue,
                                                   ImageQueueToDescriptor,
                                                   keypointQueueToDescriptor,
                                                   ImageQueueToDisplay,
                                                   keypointQueueToPose);


    SharedFifo<std::vector< sptrnms::shared_ptr<SolARKeypoint> >  >  keypointQueueToDescriptorVect0;
    SharedFifo<std::vector< sptrnms::shared_ptr<SolARKeypoint> >  >  keypointQueueToDescriptorVect1;
    SharedFifo<SRef<SolARImage>>  ImageQueueToDescriptorVect0;
    SharedFifo<SRef<SolARImage>>  ImageQueueToDescriptorVect1;
    SolARKeypointSplitterTask keypointSplitterTask(2,keypointQueueToDescriptor,ImageQueueToDescriptor,
                                                     keypointQueueToDescriptorVect0,keypointQueueToDescriptorVect1,
                                                     ImageQueueToDescriptorVect0,ImageQueueToDescriptorVect1);


    SharedFifo< sptrnms::shared_ptr<SolAR::SolARDescriptorBuffer>  > descriptorQueueVect0;
    SolARDescriptorsExtractorTask descriptorsExtractorTask0(descriptorsExtractor0,ImageQueueToDescriptorVect0,keypointQueueToDescriptorVect0,descriptorQueueVect0);

    SharedFifo< sptrnms::shared_ptr<SolAR::SolARDescriptorBuffer>  > descriptorQueueVect1;
    SolARDescriptorsExtractorTask descriptorsExtractorTask1(descriptorsExtractor1,ImageQueueToDescriptorVect1,keypointQueueToDescriptorVect1,descriptorQueueVect1);


    SharedFifo< std::vector<SolAR::DescriptorMatcher::Match>> matchQueue;

    SolARDescriptorMatcherTask descriptorMatcherTask(matcher,refDescriptors,descriptorQueueVect0,descriptorQueueVect1,matchQueue);


    SharedFifo<SolARPose> poseQueue;
    SolARPoseHomographyTask poseHomographyTask(homographyFinder,matchQueue,keypointQueueToPose,poseQueue);


    cameraTask.start();
    keypointDetectorTask.start();
    keypointSplitterTask.start();
    descriptorsExtractorTask0.start();
    descriptorsExtractorTask1.start();
    descriptorMatcherTask.start();
    poseHomographyTask.start();

    int count=0;
    // to count the average number of processed frames per seconds
    clock_t start,end;
    start= clock();

    while(++count<500){

    SRef<SolARImage> outputImage;
    std::vector< sptrnms::shared_ptr<SolARKeypoint> >   keypoints;
    SRef<SolARImage>  image;
    std::vector<SolAR::DescriptorMatcher::Match> matches;
    SolARPose pose;

    outputImage=ImageQueueToDisplay.pop();

#if 0
    matches=matchQueue.pop();
    keypoints=keypointQueueToPose.pop();
#else
    pose=poseQueue.pop();
#endif
    //draw cube
    if(pose.getPoseMatrix()(3,3)!=0.0)
        homographyFinder->overlay3DProjectedPoint( pose,camera, refImage, outputImage);
    if(imageViewer->display("camera",outputImage,"e")==SolAR::FrameworkReturnCode::_STOP){
        break;
        }
    }



    end= clock();
    double duration=double(end - start) / CLOCKS_PER_SEC;
    printf ("\n\nElasped time is %.2lf seconds.\n",duration );
    printf (  "Number of processed frame per second : %8.2f\n",count/duration );

    std::cout << std::flush;

     cameraTask.stop();
     keypointDetectorTask.stop();
     descriptorsExtractorTask0.stop();
     descriptorsExtractorTask1.stop();
     descriptorMatcherTask.stop();
     poseHomographyTask.stop();

    return 0;

}



int printHelp(){
        printf(" usage :\n");
        printf(" exe targetNaturalImagemarkerFile CameraCalibrationFile videoFile(if videoFile missing, real-time camera mode)\n");
        return 1;
}

int main(int argc, char *argv[]){

    if(argc==3 || argc==4){
        //matching between the video stream and a reference image.
        return run3(argc,argv);
    }
    else
        return(printHelp());
}



