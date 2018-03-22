cmake_minimum_required(VERSION 2.8)

##################################################
project("SolARCameraCalibration")
set (SOURCES SolARCameraCalibrationOpenCVTest.cpp)
##################################################

# various macros
include("macros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()

# define the list of files to copy to build directory
if (UNIX)
	set (LIBPREFIX "lib")
	set (LIBEXTENSION "so")
	set (OPENCV_VERSION "")
endif(UNIX)
if (WIN32)
	set (LIBPREFIX "")
	set (LIBEXTENSION "dll")
	set (OPENCV_VERSION "320")	
endif(WIN32)

set(FILES_TO_COPY

	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}opencv_core${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}opencv_calib3d${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}opencv_features2d${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}opencv_flann${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}opencv_highgui${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}opencv_imgcodecs${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}opencv_imgproc${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}opencv_videoio${OPENCV_VERSION}.${LIBEXTENSION}


	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}boost_filesystem.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}boost_system.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}boost_timer.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}boost_log.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}boost_chrono.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}boost_thread.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}boost_date_time.${LIBEXTENSION}

	$ENV{BCOMDEVROOT}/bcomBuild/SolARModuleOpencv/0.3.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}SolARModuleOpencv.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/bcomBuild/SolARModuleNonFreeOpencv/0.3.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}SolARModuleNonFreeOpencv.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/0.3.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}SolARFramework.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/bcomBuild/SolARModuleTools/0.3.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}SolARModuleTools.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/xpcf/1.0.0/lib/x86_64/shared/${CMAKE_DEBUG_OR_RELEASE}/${LIBPREFIX}xpcf.${LIBEXTENSION}
	)
# define targets (library, install and uninstall)
defineTargets("executable" "${FILES_TO_COPY}")

