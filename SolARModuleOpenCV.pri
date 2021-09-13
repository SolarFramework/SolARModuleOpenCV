HEADERS += interfaces/SolAR2D3DcorrespondencesFinderOpencv.h \
    $$PWD/interfaces/SolARImageFilterWallisOpencv.h \
    interfaces/SolAR2DOverlayOpencv.h \
    interfaces/SolAR3DOverlayBoxOpencv.h \
    interfaces/SolARBaseCameraOpencv.h \
    interfaces/SolARCameraCalibrationOpencv.h \
    interfaces/SolARCameraOpencv.h \
    interfaces/SolARContoursExtractorOpencv.h \
    interfaces/SolARContoursFilterBinaryMarkerOpencv.h \
    interfaces/SolARCornerRefinementOpencv.h \
    interfaces/SolARDescriptorMatcherHammingBruteForceOpencv.h \
    interfaces/SolARDescriptorMatcherKNNOpencv.h \
    interfaces/SolARDescriptorMatcherRadiusOpencv.h \
    interfaces/SolARDescriptorMatcherGeometricOpencv.h \
    interfaces/SolARDescriptorMatcherRegionOpencv.h \
    interfaces/SolARDescriptorsExtractorAKAZE2Opencv.h \
    interfaces/SolARDescriptorsExtractorAKAZEOpencv.h \
    interfaces/SolARDescriptorsExtractorORBOpencv.h \
    interfaces/SolARDescriptorsExtractorSBPatternOpencv.h \
    interfaces/SolARDescriptorsExtractorSIFTOpencv.h \
    interfaces/SolARDeviceDataLoader.h \
    interfaces/SolARFiducialMarkerLoaderOpencv.h \
    interfaces/SolARFundamentalMatrixEstimationOpencv.h \
    interfaces/SolARGeometricMatchesFilterOpencv.h \
    interfaces/SolARHomographyEstimationOpencv.h \
    interfaces/SolARHomographyMatrixDecomposerOpencv.h \
    interfaces/SolARImageConvertorOpencv.h \
    interfaces/SolARImageConvertorUnity.h \
    interfaces/SolARImageFilterAdaptiveBinaryOpencv.h \
    interfaces/SolARImageFilterBinaryOpencv.h \
    interfaces/SolARImageFilterBlurOpencv.h \
    interfaces/SolARImageFilterDilateOpencv.h \
    interfaces/SolARImageFilterErodeOpencv.h \
    interfaces/SolARImageLoaderOpencv.h \
    interfaces/SolARImagesAsCameraOpencv.h \
    interfaces/SolARImageViewerOpencv.h \
    interfaces/SolARKeypointDetectorOpencv.h \
    interfaces/SolARKeypointDetectorRegionOpencv.h \
    interfaces/SolARMapFusionOpencv.h \
    interfaces/SolARImageMarkerLoaderOpencv.h \
    interfaces/SolARMatchesOverlayOpencv.h \
    interfaces/SolARModuleOpencv_traits.h \
    interfaces/SolAROpencvAPI.h \
    interfaces/SolAROpenCVHelper.h \
    interfaces/SolAROpticalFlowPyrLKOpencv.h \
    interfaces/SolARPerspectiveControllerOpencv.h \
    interfaces/SolARPoseEstimationPlanarPointsOpencv.h \
    interfaces/SolARPoseEstimationPnpOpencv.h \
    interfaces/SolARPoseEstimationSACPnpOpencv.h \
    interfaces/SolARPoseFinderFrom2D2DOpencv.h \
    interfaces/SolARProjectOpencv.h \
	interfaces/SolARStereo2DPointsRectificationOpencv.h \
    interfaces/SolARStereoCalibrationOpencv.h \
    interfaces/SolARStereoDescriptorMatcherOpencv.h \
    interfaces/SolARStereoImageRectificationOpencv.h \
    interfaces/SolARSVDFundamentalMatrixDecomposerOpencv.h\
    interfaces/SolARUndistortPointsOpencv.h \
    interfaces/SolARUnprojectPlanarPointsOpencv.h \
    interfaces/SolARSVDTriangulationOpencv.h \
    interfaces/SolARVideoAsCameraOpencv.h \
    src/AKAZE2/AKAZEConfig.h \
    src/AKAZE2/AKAZEFeatures.h \
    src/AKAZE2/fed.h \
    src/AKAZE2/nldiffusion_functions.h \
    src/AKAZE2/TEvolution.h \
    src/AKAZE2/utils.h

SOURCES +=  src/AKAZE2/akaze.cpp \
    $$PWD/src/SolARImageFilterWallisOpencv.cpp \
    $$PWD/src/SolARImageMarkerLoaderOpencv.cpp \
    src/AKAZE2/AKAZEFeatures.cpp \
    src/AKAZE2/fed.cpp \
    src/AKAZE2/nldiffusion_functions.cpp \
    src/SolAR2D3DcorrespondencesFinderOpencv.cpp \
    src/SolAR2DOverlayOpencv.cpp \
    src/SolAR3DOverlayBoxOpencv.cpp \
    src/SolARBaseCameraOpencv.cpp \
    src/SolARCameraCalibrationOpencv.cpp \
    src/SolARCameraOpencv.cpp \
    src/SolARContoursExtractorOpencv.cpp \
    src/SolARContoursFilterBinaryMarkerOpencv.cpp \
    src/SolARCornerRefinementOpencv.cpp \
    src/SolARDescriptorMatcherHammingBruteForceOpencv.cpp \
    src/SolARDescriptorMatcherKNNOpencv.cpp \
    src/SolARDescriptorMatcherRadiusOpencv.cpp \
    src/SolARDescriptorMatcherGeometricOpencv.cpp \
    src/SolARDescriptorMatcherRegionOpencv.cpp \
    src/SolARDescriptorsExtractorAKAZE2Opencv.cpp \
    src/SolARDescriptorsExtractorAKAZEOpencv.cpp \
    src/SolARDescriptorsExtractorORBOpencv.cpp \
    src/SolARDescriptorsExtractorSBPatternOpencv.cpp \
    src/SolARDescriptorsExtractorSIFTOpencv.cpp \
    src/SolARDeviceDataLoader.cpp \
    src/SolARFiducialMarkerLoaderOpencv.cpp \
    src/SolARFundamentalMatrixEstimationOpencv.cpp \
    src/SolARGeometricMatchesFilterOpencv.cpp \
    src/SolARHomographyEstimationOpencv.cpp \
    src/SolARHomographyMatrixDecomposerOpencv.cpp \
    src/SolARImageConvertorOpencv.cpp \
    src/SolARImageConvertorUnity.cpp \
    src/SolARImageFilterAdaptiveBinaryOpencv.cpp \
    src/SolARImageFilterBinaryOpencv.cpp \
    src/SolARImageFilterBlurOpencv.cpp \
    src/SolARImageFilterDilateOpencv.cpp \
    src/SolARImageFilterErodeOpencv.cpp \
    src/SolARImageLoaderOpencv.cpp \
    src/SolARImagesAsCameraOpencv.cpp \
    src/SolARImageViewerOpencv.cpp \
    src/SolARKeypointDetectorOpencv.cpp \
    src/SolARKeypointDetectorRegionOpencv.cpp \
    src/SolARMapFusionOpencv.cpp \
    src/SolARMatchesOverlayOpencv.cpp \
    src/SolARModuleOpencv.cpp \
    src/SolAROpenCVHelper.cpp \
    src/SolAROpticalFlowPyrLKOpencv.cpp \
    src/SolARPerspectiveControllerOpencv.cpp \
    src/SolARPoseEstimationPlanarPointsOpencv.cpp \
    src/SolARPoseEstimationPnpOpencv.cpp \
    src/SolARPoseEstimationSACPnpOpencv.cpp \
    src/SolARPoseFinderFrom2D2DOpencv.cpp \
    src/SolARProjectOpencv.cpp \
	src/SolARStereo2DPointsRectificationOpencv.cpp \
    src/SolARStereoCalibrationOpencv.cpp \
    src/SolARStereoDescriptorMatcherOpencv.cpp \
    src/SolARStereoImageRectificationOpencv.cpp \
    src/SolARSVDFundamentalMatrixDecomposerOpencv.cpp \
    src/SolARSVDTriangulationOpencv.cpp \
    src/SolARUndistortPointsOpencv.cpp \
    src/SolARUnprojectplanarPointsOpencv.cpp \
    src/SolARVideoAsCameraOpencv.cpp




