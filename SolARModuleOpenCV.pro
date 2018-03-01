## remove Qt dependencies
QT       -= core gui
CONFIG -= qt

## global defintions : target lib name, version
TARGET = SolARModuleOpenCV
INSTALLSUBDIR = bcomBuild
FRAMEWORK = $$TARGET
VERSION=0.3.0

DEFINES += MYVERSION=$${VERSION}
DEFINES += TEMPLATE_LIBRARY
CONFIG += Cpp11
CONFIG += c++11


CONFIG(debug,debug|release) {
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    DEFINES += _NDEBUG=1
    DEFINES += NDEBUG=1
}


PROJECTDEPLOYDIR = $$(BCOMDEVROOT)/$${INSTALLSUBDIR}/$${FRAMEWORK}/$${VERSION}
DEPENDENCIESCONFIG = shared

include ($$(BCOMDEVROOT)/builddefs/qmake/templatelibconfig.pri)

## DEFINES FOR MSVC/INTEL C++ compilers
msvc {
DEFINES += "_BCOM_SHARED=__declspec(dllexport)"
}

INCLUDEPATH += interfaces/

HEADERS += interfaces/SolARCameraOpencv.h \
interfaces/SolARImageConvertorOpencv.h \
interfaces/SolARImageLoaderOpencv.h \
interfaces/SolARImageFilterOpencv.h \
interfaces/SolARImageViewerOpencv.h \
interfaces/SolARKeypointDetectorOpencv.h \
interfaces/SolAROpenCVHelper.h \
interfaces/SolARImageFilterOpencv.h \
interfaces/SolAROpencvAPI.h \
interfaces/SolARCameraCalibrationOpencv.h \
interfaces/SolARMarker2DNaturalImageOpencv.h \
interfaces/SolARContoursExtractorOpencv.h \
interfaces/SolARPerspectiveControllerOpencv.h \
interfaces/SolARMarker2DSquaredBinaryOpencv.h \
interfaces/SolARContoursFilterBinaryMarkerOpencv.h \
interfaces/SolARDescriptorsExtractorSBPatternOpencv.h \
interfaces/SolARDescriptorsExtractorAKAZEOpencv.h \
interfaces/SolARDescriptorsExtractorORBOpencv.h \
interfaces/SolARHomographyEstimationOpencv.h \
interfaces/SolARPoseEstimationOpencv.h \
interfaces/SolARDescriptorMatcherHammingBruteForceOpencv.h \
interfaces/SolARDescriptorMatcherKNNOpencv.h \
interfaces/SolARDescriptorMatcherRadiusOpencv.h \
interfaces/SolARSideBySideOverlayOpencv.h \
interfaces/SolAR3DOverlayOpencv.h \
interfaces/SolAR2DOverlayOpencv.h \
interfaces/SolARModuleManagerOpencv.h

SOURCES += src/SolARModuleOpencv.cpp \
    src/SolARKeypointDetectorOpencv.cpp \
    src/SolARImageLoaderOpencv.cpp \
    src/SolARImageConvertorOpencv.cpp \
    src/SolARImageFilterOpencv.cpp \
    src/SolARImageViewerOpencv.cpp \
    src/SolARCameraOpencv.cpp \
    src/SolAROpenCVHelper.cpp \
    src/SolARCameraCalibrationOpencv.cpp \
    src/SolARMarker2DNaturalImageOpencv.cpp \
    src/SolARContoursExtractorOpencv.cpp \
    src/SolARPerspectiveControllerOpencv.cpp \
    src/SolARMarker2DSquaredBinaryOpencv.cpp \
    src/SolARContoursFilterBinaryMarkerOpencv.cpp \
    src/SolARDescriptorsExtractorSBPatternOpencv.cpp \
    src/SolARDescriptorsExtractorAKAZEOpencv.cpp \
    src/SolARDescriptorsExtractorORBOpencv.cpp \
    src/SolARPoseEstimationOpencv.cpp \
    src/SolARDescriptorMatcherHammingBruteForceOpencv.cpp \
    src/SolARDescriptorMatcherKNNOpencv.cpp \
    src/SolARDescriptorMatcherRadiusOpencv.cpp \
    src/SolARSideBySideOverlayOpencv.cpp \
    src/SolAR2DOverlayOpencv.cpp \
    src/SolAR3DOverlayOpencv.cpp \
    src/SolARModuleManagerOpencv.cpp \
    src/SolARHomographyEstimationOpencv.cpp

unix {
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
    QMAKE_CXXFLAGS += -wd4250 -wd4251 -wd4244 -wd4275
}

header_files.path = $${PROJECTDEPLOYDIR}/interfaces
header_files.files = $$files($${PWD}/interfaces/*.h*)

xpcf_xml_files.path = $${PROJECTDEPLOYDIR}
xpcf_xml_files.files=$$files($${PWD}/xpcf*.xml)

INSTALLS += header_files
INSTALLS += xpcf_xml_files
