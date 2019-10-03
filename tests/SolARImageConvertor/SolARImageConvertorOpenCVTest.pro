TARGET = SolARImageConvertorOpenCVTest
VERSION=0.6.0

CONFIG += c++11
CONFIG -= qt
CONFIG += console

DEFINES += MYVERSION=$${VERSION}

CONFIG(debug,debug|release) {
    TARGETDEPLOYDIR = $${PWD}\..\bin\debug
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    TARGETDEPLOYDIR = $${PWD}\..\bin\release
    DEFINES += _NDEBUG=1
    DEFINES += NDEBUG=1
}

win32:CONFIG -= static
win32:CONFIG += shared

DEPENDENCIESCONFIG = shared install_recurse

## Configuration for Visual Studio to install binaries and dependencies. Work also for QT Creator by replacing QMAKE_INSTALL
PROJECTCONFIG = QTVS

#NOTE : CONFIG as staticlib or sharedlib, DEPENDENCIESCONFIG as staticlib or sharedlib, QMAKE_TARGET.arch and PROJECTDEPLOYDIR MUST BE DEFINED BEFORE templatelibconfig.pri inclusion
include ($$(REMAKEN_RULES_ROOT)/qmake/templateappconfig.pri)

SOURCES += \
    main.cpp


unix {
LIBS += -ldl
}

macx {
    QMAKE_MAC_SDK= macosx
    QMAKE_CXXFLAGS += -fasm-blocks -x objective-c++
}

win32 {
    QMAKE_LFLAGS += /MACHINE:X64
    DEFINES += WIN64 UNICODE _UNICODE
    QMAKE_COMPILER_DEFINES += _WIN64

    # Windows Kit (msvc2013 64)
    LIBS += -L$$(WINDOWSSDKDIR)lib/winv6.3/um/x64 -lshell32 -lgdi32 -lComdlg32
    INCLUDEPATH += $$(WINDOWSSDKDIR)lib/winv6.3/um/x64
}

configfile.path = $${TARGETDEPLOYDIR}
configfile.file = $$files{$${PWD}/conf_ImageConvertor.xml}
INSTALLS += configfile

#NOTE : Must be placed at the end of the .pro
include ($$(REMAKEN_RULES_ROOT)/qmake/remaken_install_target.pri))
