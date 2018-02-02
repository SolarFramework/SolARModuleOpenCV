QT += core
QT -= gui

CONFIG += c++11

TARGET = withThread
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

INCLUDEPATH += $$(BCOMDEVROOT)/bcomBuild/SolARContainerOpenCV/1.0.0/interfaces
INCLUDEPATH += $$(BCOMDEVROOT)/bcomBuild/SolARFramework/1.0.0/interfaces
INCLUDEPATH += $$(BCOMDEVROOT)/bcomBuild/xpcf/1.0.0/interfaces
INCLUDEPATH += $$(BCOMDEVROOT)/thirdParties/boost/1.64.0/interfaces
INCLUDEPATH += "$$(BCOMDEVROOT)/thirdParties/opencv/3.2.0/interfaces/"
INCLUDEPATH += "$$(BCOMDEVROOT)/bcomBuild/SolARCore/1.0.0/interfaces"


SOURCES += \
    SolARMatcherOpencvHomographyTest.cpp
# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS +=


CONFIG += c++11
CONFIG -= qt
CONFIG += console

DEFINES += MYVERSION=$${VERSION}
DEFINES += BOOST_LOG_DYN_LINK

CONFIG(debug,debug|release) {
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    DEFINES += NDEBUG=1
}

win32:CONFIG -= static
win32:CONFIG += shared
DEPENDENCIESCONFIG = sharedlib
#NOTE : CONFIG as staticlib or sharedlib, QMAKE_TARGET.arch and PROJECTDEPLOYDIR MUST BE DEFINED BEFORE templatelibconfig.pri inclusion
include (../../../builddefs/qmake/templateappconfig.pri)

unix {
}

macx {
    QMAKE_MAC_SDK= macosx
    QMAKE_CXXFLAGS += -fasm-blocks  -x objective-c++
}

win32 {
    QMAKE_LFLAGS += /MACHINE:X64
    DEFINES += WIN64 UNICODE _UNICODE
    QMAKE_COMPILER_DEFINES += _WIN64

    # Windows Kit (msvc2013 64)
    LIBS += -L$$(WINDOWSSDKDIR)lib/winv6.3/um/x64 -lshell32 -lgdi32 -lComdlg32
    INCLUDEPATH += $$(WINDOWSSDKDIR)lib/winv6.3/um/x64

}
