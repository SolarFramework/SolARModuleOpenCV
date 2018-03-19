cmake_minimum_required(VERSION 2.8)

##################################################
set (VERSION_NUMBER "0.3.1")
project("SolARModuleOpenCV")
set (SOURCES 
	src/SolARModuleOpencv.cpp
	src/SolARKeypointDetectorOpencv.cpp
	src/SolARImageLoaderOpencv.cpp
	src/SolARImageConvertorOpencv.cpp
	src/SolARImageFilterOpencv.cpp
	src/SolARImageViewerOpencv.cpp
	src/SolARCameraOpencv.cpp
	src/SolAROpenCVHelper.cpp
	src/SolARCameraCalibrationOpencv.cpp
	src/SolARMarker2DNaturalImageOpencv.cpp
	src/SolARContoursExtractorOpencv.cpp
	src/SolARPerspectiveControllerOpencv.cpp
	src/SolARMarker2DSquaredBinaryOpencv.cpp
	src/SolARContoursFilterBinaryMarkerOpencv.cpp
	src/SolARDescriptorsExtractorSBPatternOpencv.cpp
	src/SolARDescriptorsExtractorAKAZEOpencv.cpp
	src/SolARDescriptorsExtractorORBOpencv.cpp
	src/SolARPoseEstimationOpencv.cpp
	src/SolARDescriptorMatcherHammingBruteForceOpencv.cpp
	src/SolARDescriptorMatcherKNNOpencv.cpp
	src/SolARDescriptorMatcherRadiusOpencv.cpp
	src/SolARSideBySideOverlayOpencv.cpp
	src/SolAR2DOverlayOpencv.cpp
	src/SolAR3DOverlayOpencv.cpp
	src/SolARModuleManagerOpencv.cpp
	src/SolARHomographyEstimationOpencv.cpp
	src/SolARDescriptorsExtractorAKAZE2Opencv.cpp
	src/AKAZE2/akaze.cpp
	src/AKAZE2/AKAZEFeatures.cpp
	src/AKAZE2/fed.cpp
	src/AKAZE2/nldiffusion_functions.cpp
	)
##################################################

# various macros
include("macros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()
# define targets (type library)
defineTargets("library")


