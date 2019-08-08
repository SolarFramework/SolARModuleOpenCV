## remove Qt dependencies
QT       -= core gui
CONFIG -= qt

## global defintions : target lib name, version
TARGET = SolARModuleOpenCV
FRAMEWORK = $$TARGET
VERSION=0.5.2

DEFINES += MYVERSION=$${VERSION}
DEFINES += TEMPLATE_LIBRARY
CONFIG += c++1z


CONFIG(debug,debug|release) {
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    DEFINES += _NDEBUG=1
    DEFINES += NDEBUG=1
}


DEPENDENCIESCONFIG = shared recurse

include (../builddefs/qmake/templatelibconfig.pri)

## DEFINES FOR MSVC/INTEL C++ compilers
msvc {
DEFINES += "_BCOM_SHARED=__declspec(dllexport)"
}

INCLUDEPATH += interfaces/

# my ssba path files:

#INCLUDEPATH += "D:/AmineLib/SSBA/"
#LIBS += "D:/AmineLib/SSBA/build/Release"

HEADERS += interfaces/SolARCameraOpencv.h \
    interfaces/SolARBaseCameraOpencv.h \
    interfaces/SolARImageConverterOpencv.h \
    interfaces/SolARImageConverterUnity.h \
    interfaces/SolARImageLoaderOpencv.h \
    interfaces/SolARImageViewerOpencv.h \
    interfaces/SolARKeypointDetectorOpencv.h \
    interfaces/SolARKeypointDetectorRegionOpencv.h \
    interfaces/SolAROpenCVHelper.h \
    interfaces/SolAROpencvAPI.h \
    interfaces/SolARCameraCalibrationOpencv.h \
    interfaces/SolARMarker2DNaturalImageOpencv.h \
    interfaces/SolARContoursExtractorOpencv.h \
    interfaces/SolARPerspectiveControllerOpencv.h \
    interfaces/SolARProjectOpencv.h \
    interfaces/SolARUnprojectPlanarPointsOpencv.h \
    interfaces/SolARMarker2DSquaredBinaryOpencv.h \
    interfaces/SolARContoursFilterBinaryMarkerOpencv.h \
    interfaces/SolARDescriptorsExtractorSBPatternOpencv.h \
    interfaces/SolARDescriptorsExtractorAKAZEOpencv.h \
    interfaces/SolARDescriptorsExtractorAKAZE2Opencv.h \
    interfaces/SolARDescriptorsExtractorORBOpencv.h \
    interfaces/SolARHomographyEstimationOpencv.h \
    interfaces/SolARDescriptorMatcherHammingBruteForceOpencv.h \
    interfaces/SolARDescriptorMatcherKNNOpencv.h \
    interfaces/SolARDescriptorMatcherRadiusOpencv.h \
    interfaces/SolARFundamentalMatrixEstimationOpencv.h \
    interfaces/SolARSVDFundamentalMatrixDecomposerOpencv.h\
    interfaces/SolARPoseEstimationPlanarPointsOpencv.h \
    interfaces/SolARPoseEstimationPnpEPFL.h \
    interfaces/SolARPoseEstimationPnpOpencv.h \
    interfaces/SolARPoseEstimationSACPnpOpencv.h \
    interfaces/SolARGeometricMatchesFilterOpencv.h \
    interfaces/SolAR2DOverlayOpencv.h \
    interfaces/SolARSVDTriangulationOpencv.h \
    interfaces/SolAROpticalFlowPyrLKOpencv.h \
    src/AKAZE2/AKAZEConfig.h \
    src/AKAZE2/AKAZEFeatures.h \
    src/AKAZE2/fed.h \
    src/AKAZE2/nldiffusion_functions.h \
    src/AKAZE2/TEvolution.h \
    src/AKAZE2/utils.h \
    interfaces/SolARModuleOpencv_traits.h \
    interfaces/SolARImageFilterAdaptiveBinaryOpencv.h \
    interfaces/SolARImageFilterBinaryOpencv.h \
    interfaces/SolARImageFilterBlurOpencv.h \
    interfaces/SolARImageFilterDilateOpencv.h \
    interfaces/SolAR2D3DcorrespondencesFinderOpencv.h \
    interfaces/SolARImageFilterErodeOpencv.h \
    interfaces/SolARVideoAsCameraOpencv.h \
    interfaces/SolARImagesAsCameraOpencv.h \
    interfaces/SolAR3DOverlayBoxOpencv.h \
    interfaces/SolARHomographyMatrixDecomposerOpencv.h \
    interfaces/SolARPoseFinderFrom2D2DOpencv.h \
    interfaces/SolARMatchesOverlayOpencv.h \
    interfaces/SolARUndistortPointsOpencv.h 

SOURCES += src/SolARModuleOpencv.cpp \
    src/SolARBaseCameraOpencv.cpp \
    src/SolARKeypointDetectorOpencv.cpp \
    src/SolARKeypointDetectorRegionOpencv.cpp \
    src/SolARImageLoaderOpencv.cpp \
    src/SolARImageConverterOpencv.cpp \
    src/SolARImageConverterUnity.cpp \
    src/SolARImageViewerOpencv.cpp \
    src/SolARCameraOpencv.cpp \
    src/SolAROpenCVHelper.cpp \
    src/SolARCameraCalibrationOpencv.cpp \
    src/SolARMarker2DNaturalImageOpencv.cpp \
    src/SolARContoursExtractorOpencv.cpp \
    src/SolARPerspectiveControllerOpencv.cpp \
    src/SolARProjectOpencv.cpp \
    src/SolARUnprojectplanarPointsOpencv.cpp \
    src/SolARMarker2DSquaredBinaryOpencv.cpp \
    src/SolARContoursFilterBinaryMarkerOpencv.cpp \
    src/SolARDescriptorsExtractorSBPatternOpencv.cpp \
    src/SolARDescriptorsExtractorAKAZEOpencv.cpp \
    src/SolARDescriptorsExtractorORBOpencv.cpp \
    src/SolARDescriptorMatcherHammingBruteForceOpencv.cpp \
    src/SolARDescriptorMatcherKNNOpencv.cpp \
    src/SolARDescriptorMatcherRadiusOpencv.cpp \
    src/SolARGeometricMatchesFilterOpencv.cpp \
    src/SolAR2DOverlayOpencv.cpp \
    src/SolARHomographyEstimationOpencv.cpp \
    src/SolARPoseEstimationPlanarPointsOpencv.cpp \
    src/SolARPoseEstimationPnpEPFL.cpp \
    src/SolARPoseEstimationPnpOpencv.cpp \
    src/SolARPoseEstimationSACPnpOpencv.cpp \
    src/SolARDescriptorsExtractorAKAZE2Opencv.cpp \
    src/AKAZE2/akaze.cpp \
    src/AKAZE2/AKAZEFeatures.cpp \
    src/AKAZE2/fed.cpp \
    src/AKAZE2/nldiffusion_functions.cpp \
    src/SolARSVDTriangulationOpencv.cpp \
    src/SolAROpticalFlowPyrLKOpencv.cpp \
    src/SolARFundamentalMatrixEstimationOpencv.cpp \
    src/SolARSVDFundamentalMatrixDecomposerOpencv.cpp \
    src/SolARImageFilterBinaryOpencv.cpp \
    src/SolARImageFilterAdaptiveBinaryOpencv.cpp \
    src/SolARImageFilterBlurOpencv.cpp \
    src/SolARImageFilterDilateOpencv.cpp \
    src/SolAR2D3DcorrespondencesFinderOpencv.cpp \
    src/SolARImageFilterErodeOpencv.cpp \
    src/SolARVideoAsCameraOpencv.cpp \
    src/SolARImagesAsCameraOpencv.cpp \
    src/SolAR3DOverlayBoxOpencv.cpp \
    src/SolARHomographyMatrixDecomposerOpencv.cpp \
    src/SolARPoseFinderFrom2D2DOpencv.cpp \
    src/SolARMatchesOverlayOpencv.cpp \
    src/SolARUndistortPointsOpencv.cpp

unix {
    QMAKE_CXXFLAGS += -Wignored-qualifiers
    QMAKE_LINK=clang++
    QMAKE_CXX = clang++	
}

macx {
    DEFINES += _MACOS_TARGET_
    QMAKE_MAC_SDK= macosx
    QMAKE_CFLAGS += -mmacosx-version-min=10.7 -std=c11 #-x objective-c++
    QMAKE_CXXFLAGS += -mmacosx-version-min=10.7 -std=c11 -std=c++11 -O3 -fPIC#-x objective-c++
    QMAKE_LFLAGS += -mmacosx-version-min=10.7 -v -lstdc++
    LIBS += -lstdc++ -lc -lpthread
}

win32 {

    DEFINES += WIN64 UNICODE _UNICODE
    QMAKE_COMPILER_DEFINES += _WIN64
    QMAKE_CXXFLAGS += -wd4250 -wd4251 -wd4244 -wd4275 /Od
}

header_files.path = $${PROJECTDEPLOYDIR}/interfaces
header_files.files = $$files($${PWD}/interfaces/*.h*)

xpcf_xml_files.path = $$(HOME)/.xpcf/SolAR
xpcf_xml_files.files=$$files($${PWD}/xpcf*.xml)

INSTALLS += header_files
INSTALLS += xpcf_xml_files

