HEADERS += $$PWD/interfaces/SolAR2D3DcorrespondencesFinderOpencv.h \
    $$PWD/interfaces/SolAR2DOverlayOpencv.h \
    $$PWD/interfaces/SolAR3DOverlayBoxOpencv.h \
    $$PWD/interfaces/SolARBaseCameraOpencv.h \
    $$PWD/interfaces/SolARCameraCalibrationOpencv.h \
    $$PWD/interfaces/SolARCameraOpencv.h \
    $$PWD/interfaces/SolARContoursExtractorOpencv.h \
    $$PWD/interfaces/SolARContoursFilterBinaryMarkerOpencv.h \
    $$PWD/interfaces/SolARCornerRefinementOpencv.h \
    $$PWD/interfaces/SolARDescriptorMatcherHammingBruteForceOpencv.h \
    $$PWD/interfaces/SolARDescriptorMatcherKNNOpencv.h \
    $$PWD/interfaces/SolARDescriptorMatcherRadiusOpencv.h \
    $$PWD/interfaces/SolARDescriptorMatcherGeometricOpencv.h \
    $$PWD/interfaces/SolARDescriptorMatcherRegionOpencv.h \
    $$PWD/interfaces/SolARDescriptorsExtractorAKAZE2Opencv.h \
    $$PWD/interfaces/SolARDescriptorsExtractorAKAZEOpencv.h \
    $$PWD/interfaces/SolARDescriptorsExtractorORBOpencv.h \
    $$PWD/interfaces/SolARDescriptorsExtractorSBPatternOpencv.h \
    $$PWD/interfaces/SolARDescriptorsExtractorSIFTOpencv.h \
    $$PWD/interfaces/SolARDescriptorsExtractorFromImageOpencv.h \
    $$PWD/interfaces/SolARDeviceDataLoader.h \
    $$PWD/interfaces/SolARFiducialMarkerLoaderOpencv.h \
    $$PWD/interfaces/SolARQRCodeLoaderOpencv.h \
    $$PWD/interfaces/SolARQRCodePoseEstimatorOpencv.h \
    $$PWD/interfaces/SolARQRCodesDetectorOpencv.h \
    $$PWD/interfaces/SolARMultiQRCodesPoseEstimatorOpencv.h \
    $$PWD/interfaces/SolARFundamentalMatrixEstimationOpencv.h \
    $$PWD/interfaces/SolARGeometricMatchesFilterOpencv.h \
    $$PWD/interfaces/SolARHomographyEstimationOpencv.h \
    $$PWD/interfaces/SolARHomographyMatrixDecomposerOpencv.h \
    $$PWD/interfaces/SolARImageConvertorOpencv.h \
    $$PWD/interfaces/SolARImageConvertorUnity.h \
    $$PWD/interfaces/SolARImageFilterAdaptiveBinaryOpencv.h \
    $$PWD/interfaces/SolARImageFilterBinaryOpencv.h \
    $$PWD/interfaces/SolARImageFilterBlurOpencv.h \
    $$PWD/interfaces/SolARImageFilterDilateOpencv.h \
    $$PWD/interfaces/SolARImageFilterErodeOpencv.h \
    $$PWD/interfaces/SolARImageFilterWallisOpencv.h \
    $$PWD/interfaces/SolARImageLoaderOpencv.h \
    $$PWD/interfaces/SolARImagesAsCameraOpencv.h \
    $$PWD/interfaces/SolARImageViewerOpencv.h \
    $$PWD/interfaces/SolARKeypointDetectorOpencv.h \
    $$PWD/interfaces/SolARKeypointDetectorRegionOpencv.h \
    $$PWD/interfaces/SolARMapFusionOpencv.h \
    $$PWD/interfaces/SolARImageMarkerLoaderOpencv.h \
    $$PWD/interfaces/SolARMatchesOverlayOpencv.h \
    $$PWD/interfaces/SolARModuleOpencv_traits.h \
    $$PWD/interfaces/SolAROpencvAPI.h \
    $$PWD/interfaces/SolAROpenCVHelper.h \
    $$PWD/interfaces/SolAROpticalFlowPyrLKOpencv.h \
    $$PWD/interfaces/SolARPerspectiveControllerOpencv.h \
    $$PWD/interfaces/SolARPoseEstimationPlanarPointsOpencv.h \
    $$PWD/interfaces/SolARPoseEstimationPnpOpencv.h \
    $$PWD/interfaces/SolARPoseEstimationSACPnpOpencv.h \
    $$PWD/interfaces/SolARPoseFinderFrom2D2DOpencv.h \
    $$PWD/interfaces/SolARProjectOpencv.h \
    $$PWD/interfaces/SolARStereo2DPointsRectificationOpencv.h \
    $$PWD/interfaces/SolARStereoCalibrationOpencv.h \
    $$PWD/interfaces/SolARStereoDescriptorMatcherOpencv.h \
    $$PWD/interfaces/SolARStereoImageRectificationOpencv.h \
    $$PWD/interfaces/SolARSVDFundamentalMatrixDecomposerOpencv.h\
    $$PWD/interfaces/SolARUndistortPointsOpencv.h \
    $$PWD/interfaces/SolARUnprojectPlanarPointsOpencv.h \
    $$PWD/interfaces/SolARSVDTriangulationOpencv.h \
    $$PWD/interfaces/SolARVideoAsCameraOpencv.h \
    $$PWD/interfaces/SolARMaskOverlayOpencv.h \
    $$PWD/interfaces/features2d_akaze2.hpp \
    $$PWD/src/AKAZE2/AKAZEConfig.h \
    $$PWD/src/AKAZE2/AKAZEFeatures.h \
    $$PWD/src/AKAZE2/fed.h \
    $$PWD/src/AKAZE2/nldiffusion_functions.h \
    $$PWD/src/AKAZE2/TEvolution.h \
    $$PWD/src/AKAZE2/utils.h

SOURCES +=  $$PWD/src/AKAZE2/akaze.cpp \
    $$PWD/src/SolARImageMarkerLoaderOpencv.cpp \
    $$PWD/src/AKAZE2/AKAZEFeatures.cpp \
    $$PWD/src/AKAZE2/fed.cpp \
    $$PWD/src/AKAZE2/nldiffusion_functions.cpp \
    $$PWD/src/SolAR2D3DcorrespondencesFinderOpencv.cpp \
    $$PWD/src/SolAR2DOverlayOpencv.cpp \
    $$PWD/src/SolAR3DOverlayBoxOpencv.cpp \
    $$PWD/src/SolARBaseCameraOpencv.cpp \
    $$PWD/src/SolARCameraCalibrationOpencv.cpp \
    $$PWD/src/SolARCameraOpencv.cpp \
    $$PWD/src/SolARContoursExtractorOpencv.cpp \
    $$PWD/src/SolARContoursFilterBinaryMarkerOpencv.cpp \
    $$PWD/src/SolARCornerRefinementOpencv.cpp \
    $$PWD/src/SolARDescriptorMatcherHammingBruteForceOpencv.cpp \
    $$PWD/src/SolARDescriptorMatcherKNNOpencv.cpp \
    $$PWD/src/SolARDescriptorMatcherRadiusOpencv.cpp \
    $$PWD/src/SolARDescriptorMatcherGeometricOpencv.cpp \
    $$PWD/src/SolARDescriptorMatcherRegionOpencv.cpp \
    $$PWD/src/SolARDescriptorsExtractorAKAZE2Opencv.cpp \
    $$PWD/src/SolARDescriptorsExtractorAKAZEOpencv.cpp \
    $$PWD/src/SolARDescriptorsExtractorORBOpencv.cpp \
    $$PWD/src/SolARDescriptorsExtractorSBPatternOpencv.cpp \
    $$PWD/src/SolARDescriptorsExtractorSIFTOpencv.cpp \
    $$PWD/src/SolARDescriptorsExtractorFromImageOpencv.cpp \
    $$PWD/src/SolARDeviceDataLoader.cpp \
    $$PWD/src/SolARFiducialMarkerLoaderOpencv.cpp \
    $$PWD/src/SolARQRCodeLoaderOpencv.cpp \
    $$PWD/src/SolARQRCodePoseEstimatorOpencv.cpp \
    $$PWD/src/SolARQRCodesDetectorOpencv.cpp \
    $$PWD/src/SolARMultiQRCodesPoseEstimatorOpencv.cpp \
    $$PWD/src/SolARFundamentalMatrixEstimationOpencv.cpp \
    $$PWD/src/SolARGeometricMatchesFilterOpencv.cpp \
    $$PWD/src/SolARHomographyEstimationOpencv.cpp \
    $$PWD/src/SolARHomographyMatrixDecomposerOpencv.cpp \
    $$PWD/src/SolARImageConvertorOpencv.cpp \
    $$PWD/src/SolARImageConvertorUnity.cpp \
    $$PWD/src/SolARImageFilterAdaptiveBinaryOpencv.cpp \
    $$PWD/src/SolARImageFilterBinaryOpencv.cpp \
    $$PWD/src/SolARImageFilterBlurOpencv.cpp \
    $$PWD/src/SolARImageFilterDilateOpencv.cpp \
    $$PWD/src/SolARImageFilterErodeOpencv.cpp \
    $$PWD/src/SolARImageFilterWallisOpencv.cpp \
    $$PWD/src/SolARImageLoaderOpencv.cpp \
    $$PWD/src/SolARImagesAsCameraOpencv.cpp \
    $$PWD/src/SolARImageViewerOpencv.cpp \
    $$PWD/src/SolARKeypointDetectorOpencv.cpp \
    $$PWD/src/SolARKeypointDetectorRegionOpencv.cpp \
    $$PWD/src/SolARMapFusionOpencv.cpp \
    $$PWD/src/SolARMatchesOverlayOpencv.cpp \
    $$PWD/src/SolARModuleOpencv.cpp \
    $$PWD/src/SolAROpenCVHelper.cpp \
    $$PWD/src/SolAROpticalFlowPyrLKOpencv.cpp \
    $$PWD/src/SolARPerspectiveControllerOpencv.cpp \
    $$PWD/src/SolARPoseEstimationPlanarPointsOpencv.cpp \
    $$PWD/src/SolARPoseEstimationPnpOpencv.cpp \
    $$PWD/src/SolARPoseEstimationSACPnpOpencv.cpp \
    $$PWD/src/SolARPoseFinderFrom2D2DOpencv.cpp \
    $$PWD/src/SolARProjectOpencv.cpp \
    $$PWD/src/SolARStereo2DPointsRectificationOpencv.cpp \
    $$PWD/src/SolARStereoCalibrationOpencv.cpp \
    $$PWD/src/SolARStereoDescriptorMatcherOpencv.cpp \
    $$PWD/src/SolARStereoImageRectificationOpencv.cpp \
    $$PWD/src/SolARSVDFundamentalMatrixDecomposerOpencv.cpp \
    $$PWD/src/SolARSVDTriangulationOpencv.cpp \
    $$PWD/src/SolARUndistortPointsOpencv.cpp \
    $$PWD/src/SolARUnprojectplanarPointsOpencv.cpp \
    $$PWD/src/SolARVideoAsCameraOpencv.cpp \
    $$PWD/src/SolARMaskOverlayOpencv.cpp

!android {

HEADERS += \
    $$PWD/interfaces/SolARYOLACTSegmentationOpencv.h \
    $$PWD/interfaces/SolARFCNSegmentationOpencv.h

SOURCES += \
    $$PWD/src/SolARYOLACTSegmentationOpencv.cpp \
    $$PWD/src/SolARFCNSegmentationOpencv.cpp

}


