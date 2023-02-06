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

cmake_minimum_required (VERSION 3.1.0)

project(cvkit)

include(cmake/project_version.cmake)
include(cmake/project_dependencies.cmake)
include(GNUInstallDirs)

include(cmake/disable_psabi.cmake)

# For CMake Version >= 3.0: issue FATAL_ERROR if link dependency contains
# double-colons but is not an imported target.
if (POLICY CMP0028)
  cmake_policy(SET CMP0028 NEW)
endif ()

enable_testing()

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_BINARY_DIR}/cvkit)

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

# check for optional external packages

find_package(JPEG)
find_package(PNG)
find_package(GDAL)

set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL)

find_package(GLUT)
find_package(GLEW)

set(FLTK_SKIP_FORMS ON)
set(FLTK_SKIP_IMAGES ON)
set(FLTK_SKIP_FLUID ON)
find_package(FLTK)

if (NOT WIN32 AND NOT CYGWIN)
  find_package(X11)
  find_path(INOTIFY_DIR sys/inotify.h)

  if (INOTIFY_DIR)
    include_directories(INOTIFY_DIR)
    add_definitions(-DINCLUDE_INOTIFY)
  endif ()
endif ()

# make it possible to compile without external dependencies

if (JPEG_FOUND)
  set(USE_JPEG 1 CACHE BOOL "Use JPEG for loading and saving images")
else ()
  set(USE_JPEG 0 CACHE BOOL "Use JPEG for loading and saving images" FORCE)
endif ()

if (PNG_FOUND)
  set(USE_PNG 1 CACHE BOOL "Use PNG for loading and saving images")
else ()
  set(USE_PNG 0 CACHE BOOL "Use PNG for loading and saving images" FORCE)
endif ()

if (GDAL_FOUND)
  set(USE_GDAL 1 CACHE BOOL "Use GDAL for loading and saving images")
else ()
  set(USE_GDAL 0 CACHE BOOL "Use GDAL for loading and saving images" FORCE)
endif ()

if (OPENGL_FOUND AND (GLUT_FOUND OR FLTK_FOUND) AND GLEW_FOUND)
  set(USE_3D 1 CACHE BOOL "Compile with support for 3D visualization")

  if (GLUT_FOUND)
    set(USE_FLTK 0 CACHE BOOL "Use FLTK as replacement of GLUT")
  else ()
    set(USE_FLTK 1 CACHE BOOL "Use FLTK as replacement of GLUT" FORCE)
  endif ()
else ()
  set(USE_3D 0 CACHE BOOL "Compile with support for 3D visualization" FORCE)
  set(USE_FLTK 0 CACHE BOOL "Use FLTK as replacement of GLUT" FORCE)
endif ()

# - Use C++11 -

set(CMAKE_CXX_STANDARD 11)

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

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

# declare dynamic and static libraries

set(PROJECT_LIBRARIES gimage gmath gutil bgui gvr)

set(PROJECT_STATIC_LIBRARIES gimage_static gmath_static gutil_static bgui_static gvr_static)

# install generated version.h file
# the destination path might have to be adjusted for the project

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/project_version.h
  COMPONENT dev
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

include(cmake/configure_link_libs.cmake)

# packaging

set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Heiko Hirschmueller <heiko.hirschmueller@roboception.de>")

if (WIN32)
  include(cmake/package_win32.cmake)
else ()
#  set(CPACK_DEBIAN_PACKAGE_DEPENDS "")
  include(cmake/package_debian.cmake)
endif ()

