cmake_minimum_required(VERSION 3.7.2)

##################################################
project("SolARMarker2DFiducialOpenCVStaticTest")
set (SOURCES main.cpp)
##################################################

# various macros
include("$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/solarmacros.cmake")
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
set (BUILDCONFIG $<$<CONFIG:Debug>:debug>$<$<NOT:$<CONFIG:Debug>>:release>)

set(FILES_TO_COPY

	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}opencv_core${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}opencv_calib3d${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}opencv_features2d${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}opencv_flann${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}opencv_highgui${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}opencv_imgcodecs${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}opencv_imgproc${OPENCV_VERSION}.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/opencv/3.2.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}opencv_videoio${OPENCV_VERSION}.${LIBEXTENSION}


	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_filesystem.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_system.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_timer.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_log.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_chrono.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_thread.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/1.64.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_date_time.${LIBEXTENSION}

	$ENV{BCOMDEVROOT}/bcomBuild/SolARModuleOpenCV/0.4.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}SolARModuleOpenCV.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/0.4.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}SolARFramework.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/bcomBuild/SolARModuleTools/0.4.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}SolARModuleTools.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/xpcf/1.0.0/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}xpcf.${LIBEXTENSION}
	)
# define targets (library, install and uninstall)
defineTargets("executable" "${FILES_TO_COPY}")

