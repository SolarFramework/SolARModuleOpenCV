cmake_minimum_required(VERSION 3.7.2)

##################################################
set (VERSION_NUMBER "0.4.0")
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
    src/SolARDescriptorMatcherHammingBruteForceOpencv.cpp
    src/SolARDescriptorMatcherKNNOpencv.cpp
    src/SolARDescriptorMatcherRadiusOpencv.cpp
    src/SolARGeometricMatchesFilterOpencv.cpp
    src/SolARSideBySideOverlayOpencv.cpp
    src/SolAR2DOverlayOpencv.cpp
    src/SolAR3DOverlayOpencv.cpp
    src/SolARHomographyEstimationOpencv.cpp
    src/SolARPoseEstimationPnpEPFL.cpp
    src/SolARPoseEstimationPnpOpencv.cpp
    src/SolARDescriptorsExtractorAKAZE2Opencv.cpp
    src/AKAZE2/akaze.cpp
    src/AKAZE2/AKAZEFeatures.cpp
    src/AKAZE2/fed.cpp
    src/AKAZE2/nldiffusion_functions.cpp
    src/SolARSVDTriangulationOpencv.cpp
    src/SolARHomographyMatrixDecompositionOpencv.cpp
    src/SolARFundamentalMatrixEstimationOpencv.cpp
    src/SolARSVDFundamentalMatrixDecomposerOpencv.cpp
)

set (HEADERS
		interfaces/SolARCameraOpencv.h
		interfaces/SolARImageConvertorOpencv.h
		interfaces/SolARImageLoaderOpencv.h
		interfaces/SolARImageFilterOpencv.h
		interfaces/SolARImageViewerOpencv.h
		interfaces/SolARKeypointDetectorOpencv.h
		interfaces/SolAROpenCVHelper.h
		interfaces/SolARImageFilterOpencv.h
		interfaces/SolAROpencvAPI.h
		interfaces/SolARCameraCalibrationOpencv.h
		interfaces/SolARMarker2DNaturalImageOpencv.h
		interfaces/SolARContoursExtractorOpencv.h
		interfaces/SolARPerspectiveControllerOpencv.h
		interfaces/SolARMarker2DSquaredBinaryOpencv.h
		interfaces/SolARContoursFilterBinaryMarkerOpencv.h
		interfaces/SolARDescriptorsExtractorSBPatternOpencv.h
		interfaces/SolARDescriptorsExtractorAKAZEOpencv.h
		interfaces/SolARDescriptorsExtractorAKAZE2Opencv.h
		interfaces/SolARDescriptorsExtractorORBOpencv.h
		interfaces/SolARHomographyEstimationOpencv.h
		interfaces/SolARDescriptorMatcherHammingBruteForceOpencv.h
		interfaces/SolARDescriptorMatcherKNNOpencv.h
		interfaces/SolARDescriptorMatcherRadiusOpencv.h
		interfaces/SolARHomographyMatrixDecompositionOpencv.h
		interfaces/SolARFundamentalMatrixEstimationOpencv.h
		interfaces/SolARSVDFundamentalMatrixDecomposerOpencv.h
		interfaces/SolARPoseEstimationPnpEPFL.h
		interfaces/SolARPoseEstimationPnpOpencv.h
		interfaces/SolARGeometricMatchesFilterOpencv.h
		interfaces/SolARSideBySideOverlayOpencv.h
		interfaces/SolAR3DOverlayOpencv.h
		interfaces/SolAR2DOverlayOpencv.h
		interfaces/SolARSVDTriangulationOpencv.h
		src/AKAZE2/AKAZEConfig.h
		src/AKAZE2/AKAZEFeatures.h
		src/AKAZE2/fed.h
		src/AKAZE2/nldiffusion_functions.h
		src/AKAZE2/TEvolution.h
		src/AKAZE2/utils.h
)
##################################################

# various macros
include("$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/solarmacros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()
# define targets (type library)
defineTargets("library" "")


