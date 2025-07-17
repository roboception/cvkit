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

project(gmath CXX)

set(gmath_src
  linalg.cc
  polynomial.cc
  distortion.cc
  camera.cc
  camgeom.cc
  pose.cc
  minpack.cc
  minpack/lmdif.c
  minpack/lmder.c
  minpack/lmpar.c
  minpack/fdjac2.c
  minpack/qrfac.c
  minpack/qrsolv.c
  minpack/dpmpar.c
  minpack/enorm.c
  estimatedplane.cc
  idw.cc
  homography.cc
)

set(gmath_hh
  distortion.h
  camera.h
  camgeom.h
  pose.h
  quaternion.h
  dmatrix.h
  dvector.h
  linalg.h
  polynomial.h
  smatrix.h
  sturmchain.h
  svector.h
  minpack.h
  estimatedplane.h
  idw.h
  homography.h
)

add_library(gmath_static STATIC ${gmath_src})

add_library(gmath SHARED ${gmath_src})
target_link_libraries(gmath LINK_PRIVATE gutil)

set_target_properties(gmath PROPERTIES SOVERSION ${abiversion})

install(TARGETS gmath EXPORT PROJECTTargets COMPONENT bin
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(TARGETS gmath_static EXPORT PROJECTTargets COMPONENT dev
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${gmath_hh} COMPONENT dev DESTINATION include/gmath)
