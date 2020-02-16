TARGET = SolAROpenCVFundamentalMatrixDecomposer
VERSION=0.7.0

CONFIG += c++1z
CONFIG -= qt
CONFIG += console

DEFINES += MYVERSION=$${VERSION}


CONFIG(debug,debug|release) {
    TARGETDEPLOYDIR = $${PWD}/../bin/Debug
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    TARGETDEPLOYDIR = $${PWD}/../bin/Release
    DEFINES += _NDEBUG=1
    DEFINES += NDEBUG=1
}

DEPENDENCIESCONFIG = shared install_recurse

win32:CONFIG -= static
win32:CONFIG += shared

## Configuration for Visual Studio to install binaries and dependencies. Work also for QT Creator by replacing QMAKE_INSTALL
PROJECTCONFIG = QTVS

#NOTE : CONFIG as staticlib or sharedlib, DEPENDENCIESCONFIG as staticlib or sharedlib, QMAKE_TARGET.arch and TARGETDEPLOYDIR MUST BE DEFINED BEFORE templatelibconfig.pri inclusion
include ($$shell_quote($$shell_path($$(REMAKEN_RULES_ROOT)/qmake/templateappconfig.pri))) # Shell_quote & shell_path required for visual on windows

HEADERS += \

SOURCES += \
    main.cpp

unix {
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

configfile.path = $${TARGETDEPLOYDIR}/
configfile.files = $${PWD}/SolAROpenCVFundamentalMatrixDecomposer_conf.xml
INSTALLS += configfile

#NOTE : Must be placed at the end of the .pro
include ($$shell_quote($$shell_path($$(REMAKEN_RULES_ROOT)/qmake/remaken_install_target.pri))) # Shell_quote & shell_path required for visual on windows
