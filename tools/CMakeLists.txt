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

project(tools CXX)

# define internal static libraries

set(libs
  gimage_static
  gmath_static
  gutil_static
)

# build executables

add_executable(imgcmd imgcmd.cc)
target_link_libraries(imgcmd ${libs})

add_executable(plycmd plycmd.cc)
if (USE_3D)
  if (GLEW_FOUND)
    target_link_libraries(plycmd GLEW::GLEW)
  endif ()
  if (GLUT_FOUND)
    target_link_libraries(plycmd ${GLUT_LIBRARIES})
  endif ()
endif ()
target_link_libraries(plycmd gvr_static ${libs})

add_executable(prop2octave prop2octave.cc)
target_link_libraries(prop2octave ${libs})

install(TARGETS imgcmd plycmd prop2octave DESTINATION bin)

if (WIN32)
  add_executable(sv WIN32 sv.cc win_main.cc)
  target_link_libraries(sv bgui_static ${libs})

  if (USE_FLTK)
    target_link_libraries(sv ${FLTK_LIBRARIES})
    target_link_libraries(sv gdiplus)
  endif ()

  target_link_libraries(sv bgui_static comctl32)
  install(TARGETS sv DESTINATION bin)
elseif (X11_FOUND AND CMAKE_USE_PTHREADS_INIT)
  add_executable(sv sv.cc)
  target_link_libraries(sv bgui_static ${libs})

  if (USE_FLTK)
    target_link_libraries(sv ${FLTK_LIBRARIES})
  endif ()

  install(TARGETS sv DESTINATION bin)
  install(FILES resources/sv.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications)
  install(FILES resources/sv_32.png DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/32x32/apps RENAME sv.png)
  install(FILES resources/sv_64.png DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/64x64/apps RENAME sv.png)
  install(FILES resources/sv_128.png DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/128x128/apps RENAME sv.png)
  install(FILES resources/sv_32.png DESTINATION ${CMAKE_INSTALL_DATADIR}/pixmaps RENAME sv.png)
endif ()

if (USE_3D)
  if (USE_FLTK)
    include_directories(${FLTK_INCLUDE_DIR})
  else ()
    include_directories(${GLUT_INCLUDE_DIR})
  endif ()

  include_directories(${GLEW_INCLUDE_DIRS})
  add_definitions(-DINCLUDE_GL)

  if (WIN32)
    add_executable(plyv WIN32 plyv.cc win_main.cc)
  else ()
    add_executable(plyv plyv.cc)

    install(FILES resources/plyv.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications)
    install(FILES resources/plyv_32.png DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/32x32/apps RENAME plyv.png)
    install(FILES resources/plyv_64.png DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/64x64/apps RENAME plyv.png)
    install(FILES resources/plyv_128.png DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/128x128/apps RENAME plyv.png)
    install(FILES resources/plyv_32.png DESTINATION ${CMAKE_INSTALL_DATADIR}/pixmaps RENAME plyv.png)
  endif ()

  target_link_libraries(plyv GLEW::GLEW)

  if (USE_FLTK)
    target_link_libraries(plyv ${FLTK_LIBRARIES})
  else ()
    target_link_libraries(plyv ${GLUT_LIBRARIES})
  endif ()

  target_link_libraries(plyv gvr_static bgui_static ${libs})

  if (WIN32)
    target_link_libraries(plyv comctl32)
  endif ()

  install(TARGETS plyv DESTINATION bin)
endif ()

if (WIN32)
  if (INCLUDE_DLL)
    install(PROGRAMS ${INCLUDE_DLL} DESTINATION bin)
  endif ()

  install(FILES ../README.md RENAME README.TXT DESTINATION doc)
  install(FILES ../USAGE.md RENAME USAGE.TXT DESTINATION doc)
  install(FILES ../INSTALL.md RENAME INSTALL.TXT DESTINATION doc)
  install(FILES ../CHANGELOG.md RENAME CHANGELOG.TXT DESTINATION doc)
  install(FILES ../LICENSE RENAME LICENSE.TXT DESTINATION doc)
endif ()
