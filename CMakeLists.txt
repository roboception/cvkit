# This file is part of the Computer Vision Toolkit (cvkit).
#
# Author: Heiko Hirschmueller
#
# Copyright (c) 2014, Institute of Robotics and Mechatronics, German Aerospace Center
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required (VERSION 2.8.9)

project(cvkit)

enable_testing()

include_directories(${CMAKE_SOURCE_DIR})

# settings for NSIS (Windows) install package

set(CPACK_PACKAGE_CONTACT "Heiko Hirschmueller <heiko.hirschmueller@roboception.de>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Computer Vision Toolkit")
set(CPACK_PACKAGE_VENDOR "Institute of Robotics and Mechatronics, German Aerospace Center")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.txt")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt")
set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "9")
set(CPACK_PACKAGE_VERSION_PATCH "4")
set(CVKIT_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH})

find_program(GIT_CMD git)
if (GIT_CMD AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
  execute_process(COMMAND ${GIT_CMD} rev-parse --short HEAD
          WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
          OUTPUT_VARIABLE GIT_SHA1
          OUTPUT_STRIP_TRAILING_WHITESPACE)
  #message(STATUS "GIT_SHA1: " ${GIT_SHA1})
  execute_process(COMMAND ${GIT_CMD} describe --match "*[0-9].[0-9]*" HEAD
          WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
          OUTPUT_VARIABLE GIT_DESCRIBE
          ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  #message(STATUS "GIT_DESCRIBE: " ${GIT_DESCRIBE})

  if (GIT_DESCRIBE)
    string(REGEX REPLACE "v?([0-9.]+).*" "\\1" GIT_VERSION ${GIT_DESCRIBE})
    #message(STATUS "GIT_VERSION: " ${GIT_VERSION})
    if (NOT GIT_DESCRIBE MATCHES ${CVKIT_VERSION})
      message(WARNING "Version from git (${GIT_VERSION}) doesn't match cmake (${CVKIT_VERSION})")
    endif()
    if (GIT_DESCRIBE MATCHES ".*-g.*")
      # convert a git describe string to usable debian version, e.g. v1.7.1-7-ge324c81 to 1.7.1+7+ge324c81
      string(REGEX REPLACE "v?([0-9.]*)-([0-9]*)-([a-g0-9]*).*" "\\1+\\2+\\3" DEB_GIT_VERSION ${GIT_DESCRIBE})
      set(CPACK_PACKAGE_VERSION ${DEB_GIT_VERSION})
    else()
      # current HEAD is git tag (i.e. releaase), directly use the version
      set(CPACK_PACKAGE_VERSION ${GIT_VERSION})
    endif()
  else ()
    set(CPACK_PACKAGE_VERSION ${CVKIT_VERSION}+g${GIT_SHA1})
  endif ()
  #message(STATUS "CPACK_PACKAGE_VERSION: " ${CPACK_PACKAGE_VERSION})
endif ()

if (WIN32)
  set(CPACK_GENERATOR NSIS)

  set(CPACK_NSIS_INSTALL_ROOT "C:")
  set(CPACK_PACKAGE_INSTALL_DIRECTORY "cvkit")

  list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

  set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "
    \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\plyv.exe' '.ply' 'cvkit.plyv.ply'
    \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.pfm' 'cvkit.sv.pfm'
    \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.pgm' 'cvkit.sv.pgm'
    \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.ppm' 'cvkit.sv.ppm'
    \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.tif' 'cvkit.sv.tif'
    \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.jpg' 'cvkit.sv.jpg'
    \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.png' 'cvkit.sv.png'
    \\\${registerExtension} '\\\$INSTDIR\\\\bin\\\\sv.exe' '.vrt' 'cvkit.sv.vrt'
    System::Call 'shell32.dll::SHChangeNotify(i, i, i, i) v (0x08000000, 0, 0, 0)'
  ")

  set(CPACK_NSIS_CREATE_ICONS_EXTRA "
    CreateShortCut '\\\$SMPROGRAMS\\\\\$STARTMENU_FOLDER\\\\Example.lnk' '\\\$INSTDIR\\\\example'
    CreateShortCut '\\\$SMPROGRAMS\\\\\$STARTMENU_FOLDER\\\\Readme.lnk' '\\\$INSTDIR\\\\doc\\\\README.txt'
    CreateShortCut '\\\$SMPROGRAMS\\\\\$STARTMENU_FOLDER\\\\Usage.lnk' '\\\$INSTDIR\\\\doc\\\\USAGE.txt'
  ")

  set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "
    \\\${unregisterExtension} '.ply' 'cvkit.plyv.ply'
    \\\${unregisterExtension} '.pfm' 'cvkit.sv.pfm'
    \\\${unregisterExtension} '.pgm' 'cvkit.sv.pgm'
    \\\${unregisterExtension} '.ppm' 'cvkit.sv.ppm'
    \\\${unregisterExtension} '.tif' 'cvkit.sv.tif'
    \\\${unregisterExtension} '.jpg' 'cvkit.sv.jpg'
    \\\${unregisterExtension} '.png' 'cvkit.sv.png'
    \\\${unregisterExtension} '.vrt' 'cvkit.sv.vrt'
    System::Call 'shell32.dll::SHChangeNotify(i, i, i, i) v (0x08000000, 0, 0, 0)'
  ")

  set(CPACK_NSIS_DELETE_ICONS_EXTRA "
    Delete '\\\$SMPROGRAMS\\\\\$MUI_TEMP\\\\Readme.lnk'
    Delete '\\\$SMPROGRAMS\\\\\$MUI_TEMP\\\\Usage.lnk'
    Delete '\\\$SMPROGRAMS\\\\\$MUI_TEMP\\\\Example.lnk'
  ")
else (WIN32)
  set(CPACK_GENERATOR "DEB")

  find_program(DPKG_CMD dpkg)
  if (NOT DPKG_CMD)
    message(STATUS "Can not find dpkg in your path, default to i386.")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE i386)
  else ()
    execute_process(COMMAND "${DPKG_CMD}" --print-architecture
      OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  endif ()

  # add ~distribution-codename (e.g. ~trusty or ~xenial) to end of package version
  # if lsb_release is available, take it from there or fall back to DISTRO_CODENAME env variable
  set(DISTRO_CODENAME $ENV{DISTRO_CODENAME})
  find_program(LSB_RELEASE_CMD lsb_release)
  mark_as_advanced(LSB_RELEASE_CMD)
  if (LSB_RELEASE_CMD)
      execute_process(COMMAND "${LSB_RELEASE_CMD}" --codename --short
              OUTPUT_VARIABLE DISTRO_CODENAME
              OUTPUT_STRIP_TRAILING_WHITESPACE)
  endif ()
  if (DISTRO_CODENAME)
      set(CPACK_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION}-1~${DISTRO_CODENAME})
  else ()
      message(STATUS "Could not find lsb_release nor is DISTRO_CODENAME set.")
  endif ()

  string(TOLOWER "${PROJECT_NAME}" CPACK_PACKAGE_NAME)
  set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

  set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

  set(CPACK_DEBIAN_PACKAGE_SECTION "devel")

  # Generate postinst, postrm hooks which call ldconfig
  set(POSTINST_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/postinst")
  set(POSTRM_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/postrm")
  file(WRITE "${POSTINST_SCRIPT}" "#!/bin/sh\n\nset -e\n")
  file(WRITE "${POSTRM_SCRIPT}" "#!/bin/sh\n\nset -e\n")
  file(APPEND "${POSTINST_SCRIPT}" "if [ \"$1\" = \"configure\" ]; then\n        ldconfig\nfi\n")
  file(APPEND "${POSTRM_SCRIPT}" "if [ \"$1\" = \"remove\" ]; then\n        ldconfig\nfi\n")
  execute_process(COMMAND chmod 755 "${POSTINST_SCRIPT}" "${POSTRM_SCRIPT}")
  set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${POSTINST_SCRIPT};${POSTRM_SCRIPT}")
endif ()

include(CPack)

# export all DLL symbols if Visual studio is used for compilation and disable
# some warnings

if (MSVC)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
  add_definitions("/wd4251")
  add_definitions("/wd4996")
endif ()

# set default build type to release

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RELEASE CACHE STRING "Build type: DEBUG or RELEASE" FORCE)
endif ()

# choose to install header and libs

set(INSTALL_DEVELOPMENT 1 CACHE BOOL "Install header and libs for development with cvkit")

# check for optional external packages

find_package(JPEG)
find_package(PNG)
find_package(GDAL)
find_package(OpenGL)
find_package(GLUT)
find_package(GLEW)

if (NOT WIN32 AND NOT CYGWIN)
  find_package(X11)
  find_path(INOTIFY_DIR sys/inotify.h)

  if (INOTIFY_DIR)
    include_directories(INOTIFY_DIR)
    add_definitions(-DINCLUDE_INOTIFY)
  endif ()
endif ()

# make it possible to compile without GDAL, even if it is available

if (GDAL_FOUND)
  set(USE_GDAL 1 CACHE BOOL "Install header and libs for development with cvkit")
else ()
  set(USE_GDAL 0 CACHE BOOL "Install header and libs for development with cvkit" FORCE)
endif ()

# - Optionally compile as C++11 -

set(USE_CPP11 1 CACHE BOOL "Compile as C++11 (and use standard threads)")

if (USE_CPP11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif ()

# - Optional pthread support -

find_package(Threads)

if (CMAKE_USE_PTHREADS_INIT)
  set(INCLUDE_PTHREADS 1 CACHE BOOL "Use pthreads")

  if (INCLUDE_PTHREADS)
    add_definitions(-DINCLUDE_PTHREADS)
  endif ()
else ()
  set(INCLUDE_PTHREADS 0 CACHE BOOL "Use pthreads" FORCE)
endif ()

# - Standard definitions -

if (CMAKE_COMPILER_IS_GNUCC)
  add_definitions(-Wall)
endif ()

if (APPLE)
  add_definitions(-Wno-deprecated-declarations)
endif ()

# - Optional list of DLLs that are installed and included in package -

if (WIN32)
  add_definitions(-DWINVER=0x0600)
  add_definitions(-D_WIN32_IE=0x0600)
  add_definitions(-D_WIN32_WINNT=0x0600)
  add_definitions(-DWIN32=1)
  set(INCLUDE_DLL "" CACHE FILEPATH "List of DLLs to be installed and included in package")
endif ()

# build individual parts

add_subdirectory(doc)
add_subdirectory(gutil)
add_subdirectory(gmath)
add_subdirectory(gimage)
add_subdirectory(bgui)
add_subdirectory(gvr)

# build excecutables

add_subdirectory(tools)

# add examples

add_subdirectory(example)

# install cmake config files in lib install directory

if (INSTALL_DEVELOPMENT)
  export(TARGETS gutil gmath gimage bgui gvr
    FILE "${PROJECT_BINARY_DIR}/CVKITTargets.cmake")

  set(CONF_INCLUDE_DIRS "\${CVKIT_CMAKE_DIR}/../../include")

  configure_file(cmake/CVKITConfig.cmake.in
    "${PROJECT_BINARY_DIR}/CVKITConfig.cmake" @ONLY)

  configure_file(cmake/CVKITConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/CVKITConfigVersion.cmake" @ONLY)

  install(FILES
    "${PROJECT_BINARY_DIR}/CVKITConfig.cmake"
    "${PROJECT_BINARY_DIR}/CVKITConfigVersion.cmake"
    DESTINATION lib/cvkit)

  install(EXPORT CVKITTargets DESTINATION lib/cvkit)
endif ()
