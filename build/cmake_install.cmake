# Install script for directory: /home/atadrist/BCOM/SolAR/sources-github/sources/Modules/SolARModuleOpenCV

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "DEBUG")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0/lib/x86_64/shared/debug/libSolARModuleOpenCV.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0/lib/x86_64/shared/debug/libSolARModuleOpenCV.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0/lib/x86_64/shared/debug/libSolARModuleOpenCV.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0/lib/x86_64/shared/debug/libSolARModuleOpenCV.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0/lib/x86_64/shared/debug" TYPE SHARED_LIBRARY FILES "/home/atadrist/BCOM/SolAR/sources-github/sources/Modules/SolARModuleOpenCV/build/libSolARModuleOpenCV.so")
  if(EXISTS "$ENV{DESTDIR}/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0/lib/x86_64/shared/debug/libSolARModuleOpenCV.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0/lib/x86_64/shared/debug/libSolARModuleOpenCV.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0/lib/x86_64/shared/debug/libSolARModuleOpenCV.so"
         OLD_RPATH "/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/thirdParties/boost/1.64.0/lib/x86_64/shared/debug:/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/thirdParties/xpcf/2.0.0/lib/x86_64/shared/debug:/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARFramework/0.4.0/lib/x86_64/shared/debug:/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/thirdParties/opencv/3.2.0/lib/x86_64/shared/debug:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0/lib/x86_64/shared/debug/libSolARModuleOpenCV.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0/interfaces/")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0/interfaces" TYPE DIRECTORY FILES "/home/atadrist/BCOM/SolAR/sources-github/sources/Modules/SolARModuleOpenCV/interfaces/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0/bcom-SolARModuleOpenCV.pc")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0" TYPE FILE RENAME "bcom-SolARModuleOpenCV.pc" FILES "/home/atadrist/BCOM/SolAR/sources-github/sources/Modules/SolARModuleOpenCV/bcom-SolARModuleOpenCV.pc.in")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0/packagedependencies.txt")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/atadrist/EAFC32F3FC32B9A1/BCOMDEVROOT/linux/bcomBuild/SolARModuleOpenCV/0.4.0" TYPE FILE FILES "/home/atadrist/BCOM/SolAR/sources-github/sources/Modules/SolARModuleOpenCV/packagedependencies.txt")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/atadrist/BCOM/SolAR/sources-github/sources/Modules/SolARModuleOpenCV/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
