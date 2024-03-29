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

project(gutil CXX)

set(gutil_src
  exception.cc
  misc.cc
  parameter.cc
  properties.cc
  ${CMAKE_BINARY_DIR}/cvkit/project_version.cc
)

set(gutil_hh
  exception.h
  fixedint.h
  misc.h
  parameter.h
  proctime.h
  properties.h
  semaphore.h
  barrier.h
  thread.h
  msgqueue.h
  version.h
  ${CMAKE_BINARY_DIR}/cvkit/project_version.h
)

if (INCLUDE_PTHREADS)
  set(gutil_src ${gutil_src} thread_pthread.cc)
  set(gutil_src ${gutil_src} semaphore_pthread.cc)

  if (NOT APPLE)
    set(gutil_src ${gutil_src} barrier_pthread.cc)
  endif ()
else ()
  set(gutil_src ${gutil_src} thread_cpp11.cc)
  set(gutil_src ${gutil_src} semaphore_cpp11.cc)
  set(gutil_src ${gutil_src} barrier_cpp11.cc)
endif ()

add_library(gutil_static STATIC ${gutil_src})

if (INCLUDE_PTHREADS)
  target_link_libraries(gutil_static ${CMAKE_THREAD_LIBS_INIT})
endif ()

add_library(gutil SHARED ${gutil_src})

if (INCLUDE_PTHREADS)
  target_link_libraries(gutil LINK_PRIVATE ${CMAKE_THREAD_LIBS_INIT})
endif ()

set_target_properties(gutil PROPERTIES SOVERSION ${abiversion})

install(TARGETS gutil EXPORT PROJECTTargets COMPONENT bin
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(TARGETS gutil_static EXPORT PROJECTTargets COMPONENT dev
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${gutil_hh} COMPONENT dev DESTINATION include/gutil)
