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

project(gimage CXX)

set(gimage_src
  io.cc
  pnm_io.cc
  raw_io.cc
  analysis.cc
  view.cc
  polygon.cc
  hdr.cc
)

set(gimage_hh
  analysis.h
  arithmetic.h
  color.h
  image.h
  io.h
  paint.h
  size.h
  view.h
  compare.h
  polygon.h
  noise.h
  gauss.h
  gauss_pyramid.h
  laplace_pyramid.h
  hdr.h
)

if (USE_GDAL)
  include_directories(${GDAL_INCLUDE_DIRS})
  add_definitions(-DINCLUDE_GDAL)
  set(gimage_src ${gimage_src} gdal_io.cc)
endif (USE_GDAL)

if (USE_JPEG)
  include_directories(${JPEG_INCLUDE_DIRS})
  add_definitions(-DINCLUDE_JPEG)
  set(gimage_src ${gimage_src} jpeg_io.cc)
endif (USE_JPEG)

if (USE_PNG)
  include_directories(${PNG_INCLUDE_DIRS})
  add_definitions(-DINCLUDE_PNG)
  add_definitions(${PNG_DEFINITIONS})
  set(gimage_src ${gimage_src} png_io.cc)
endif (USE_PNG)

add_library(gimage_static STATIC ${gimage_src})
target_link_libraries(gimage_static gutil_static)

if (USE_JPEG)
  target_link_libraries(gimage_static ${JPEG_LIBRARIES})
endif ()

if (USE_PNG)
  target_link_libraries(gimage_static ${PNG_LIBRARIES})
endif ()

if (USE_GDAL)
  target_link_libraries(gimage_static ${GDAL_LIBRARIES})
endif ()

if (INSTALL_DEVELOPMENT)
  add_library(gimage SHARED ${gimage_src})
  target_link_libraries(gimage LINK_PRIVATE gmath)
  target_link_libraries(gimage LINK_PRIVATE gutil)

  if (USE_JPEG)
    target_link_libraries(gimage LINK_PRIVATE ${JPEG_LIBRARIES})
  endif ()

  if (USE_PNG)
    target_link_libraries(gimage LINK_PRIVATE ${PNG_LIBRARIES})
  endif ()

  if (USE_GDAL)
    target_link_libraries(gimage LINK_PRIVATE ${GDAL_LIBRARIES})
  endif ()

  install(TARGETS gimage gimage_static EXPORT CVKITTargets
          RUNTIME DESTINATION bin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)
  install(FILES ${gimage_hh} DESTINATION include/gimage)
endif ()
