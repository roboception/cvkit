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

project(gvr CXX)

set(gvr_src
  ply.cc
  model.cc
  pointcloud.cc
  coloredpointcloud.cc
  mesh.cc
  coloredmesh.cc
  texturedmesh.cc
  multitexturedmesh.cc
  coloredlines.cc
  cameracollection.cc
  extrusion.cc
)

set(gvr_hh
  cameracollection.h
  coloredlines.h
  coloredmesh.h
  coloredpointcloud.h
  mesh.h
  model.h
  multitexturedmesh.h
  ply.h
  pointcloud.h
  texturedmesh.h
  extrusion.h
)

if (USE_3D)
  if (USE_FLTK)
    include_directories(${FLTK_INCLUDE_DIR})
  else ()
    include_directories(${GLUT_INCLUDE_DIR})
  endif ()

  include_directories(${GLEW_INCLUDE_DIRS})
  add_definitions(-DINCLUDE_GL)
  set(gvr_src ${gvr_src}
    glmisc.cc
    glcamera.cc
    glpointcloud.cc
    glcoloredpointcloud.cc
    glmesh.cc
    glcoloredmesh.cc
    gltexturedmesh.cc
    glcoloredlines.cc
    gltext.cc
    glmain.cc
    glworld.cc
  )

  set(gvr_hh ${gvr_hh}
    glcamera.h
    glcoloredlines.h
    glcoloredmesh.h
    glcoloredpointcloud.h
    glmain.h
    glmesh.h
    glmisc.h
    globject.h
    glpointcloud.h
    gltext.h
    gltexturedmesh.h
    glworld.h
  )

  if (OPENGL_GLU_FOUND AND NOT WIN32)
    include_directories(${OPENGL_INCLUDE_DIR})
    add_definitions(-DINCLUDE_GLU)
  endif ()
endif ()

if (USE_GDAL)
  add_definitions(-DINCLUDE_GDAL)
endif ()

if (USE_JPEG)
  add_definitions(-DINCLUDE_JPEG)
endif ()

if (USE_PNG)
  include_directories(${PNG_INCLUDE_DIRS})
  add_definitions(-DINCLUDE_PNG)
  add_definitions(${PNG_DEFINITIONS})
endif ()

if (USE_FLTK)
  add_definitions(-DINCLUDE_FLTK)
endif ()

add_library(gvr_static STATIC ${gvr_src})

if (USE_PNG)
  target_link_libraries(gvr_static ${PNG_LIBRARIES})
endif ()

if (USE_3D)
  if (USE_FLTK)
    target_link_libraries(gvr_static ${FLTK_LIBRARIES})
  else ()
    target_link_libraries(gvr_static ${GLUT_glut_LIBRARY})
  endif ()

  target_link_libraries(gvr_static ${GLEW_LIBRARIES})
  target_link_libraries(gvr_static GLEW::GLEW)
  target_link_libraries(gvr_static ${OPENGL_LIBRARIES})

  if (OPENGL_GLU_FOUND)
    target_link_libraries(gvr_static ${OPENGL_glu_LIBRARY})
  endif ()

  if (WIN32)
    target_link_libraries(gvr_static winmm)
    target_link_libraries(gvr_static gdiplus)
  endif ()
endif ()

add_library(gvr SHARED ${gvr_src})
target_link_libraries(gvr LINK_PRIVATE gimage gmath gutil)

if (USE_PNG)
  target_link_libraries(gvr LINK_PRIVATE ${PNG_LIBRARIES})
endif ()

if (USE_3D)
  if (USE_FLTK)
    target_link_libraries(gvr LINK_PRIVATE ${FLTK_LIBRARIES})
  else ()
    target_link_libraries(gvr LINK_PRIVATE ${GLUT_glut_LIBRARY})
  endif ()

  target_link_libraries(gvr LINK_PRIVATE ${GLEW_LIBRARIES})
  target_link_libraries(gvr LINK_PRIVATE GLEW::GLEW)
  target_link_libraries(gvr LINK_PRIVATE ${OPENGL_LIBRARIES})

  if (OPENGL_GLU_FOUND)
    target_link_libraries(gvr LINK_PRIVATE ${OPENGL_glu_LIBRARY})
  endif ()

  if (WIN32)
    target_link_libraries(gvr LINK_PRIVATE winmm)
    target_link_libraries(gvr LINK_PRIVATE gdiplus)
  endif ()
endif ()

set_target_properties(gvr PROPERTIES SOVERSION ${abiversion})

install(TARGETS gvr EXPORT PROJECTTargets COMPONENT bin
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(TARGETS gvr_static EXPORT PROJECTTargets COMPONENT dev
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${gvr_hh} COMPONENT dev DESTINATION include/gvr)
