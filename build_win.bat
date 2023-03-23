
:: Build script for cvkit under Windows

@echo off
setlocal enabledelayedexpansion

where nmake >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
  echo This must be run in Visual Studio command prompt for x64
  exit /b 1
)

where git >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
  echo You must download and install git from: git-scm.com/download/win
  exit /b 1
)

where cmake >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
  echo You must download and install cmake from: https://cmake.org/download/
  exit /b 1
)

:: Create directories building and installing

if not exist "build\" mkdir build
cd build

if not exist "install\" mkdir install
cd install
set INSTALL_PATH=%CD%

if not exist "bin\" mkdir bin
if not exist "include\" mkdir include
if not exist "include\GL\" mkdir include\GL
if not exist "lib" mkdir lib

set LIB=%LIB%;%INSTALL_PATH%\lib

cd ..\..\..

echo ----- Clone all missing repositories -----

if not exist "FreeGLUT\" (
  git clone https://github.com/dcnieho/FreeGLUT.git -b 349a23dcc1264a76deb79962d1c90462ad0c6f50
)

if not exist "glew-2.2.0\" (
  echo You must download and unpack https://sourceforge.net/projects/glew/files/glew/2.2.0/glew-2.2.0-win32.zip/download
  exit /b 1
)

if not exist "%INSTALL_PATH%\include\GL\glew.h" (
  copy glew-2.2.0\include\GL\* %INSTALL_PATH%\include\GL
)

if not exist "%INSTALL_PATH%\bin\glew32.dll" (
  copy glew-2.2.0\bin\Release\x64\glew32.dll %INSTALL_PATH%\bin
)

if not exist "%INSTALL_PATH%\lib\glew32.lib" (
  copy glew-2.2.0\lib\Release\x64\glew32.lib %INSTALL_PATH%\lib
)

set OPT_GLEW=-DGLEW_INCLUDE_DIR="%CD%\glew-2.2.0\include" -DGLEW_SHARED_LIBRARY_RELEASE="%CD%\glew-2.2.0\lib\Release\x64\glew32.lib"

if not exist "zlib\" (
  git clone https://github.com/winlibs/zlib.git -b zlib-1.2.11
)

if not exist "libpng\" (
  git clone https://github.com/winlibs/libpng.git -b libpng-1.6.34
)

if not exist "libjpeg\" (
  git clone https://github.com/winlibs/libjpeg.git -b libjpeg-turbo-2.1.0
)

if not exist "libtiff\" (
  git clone https://gitlab.com/libtiff/libtiff.git -b v4.5.0
)

echo ----- Building FreeGLUT -----

cd FreeGLUT/freeglut/freeglut

if not exist "build\" mkdir build
cd build

if exist "build_cvkit\" (
  cd build_cvkit\
) else (
  mkdir build_cvkit\
  cd build_cvkit\
  cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="%INSTALL_PATH%" ..\..
)

nmake install
if %ERRORLEVEL% NEQ 0 exit /b 1

copy "%INSTALL_PATH%\include\GL\freeglut.h" "%INSTALL_PATH%\include\GL\glut.h"
set OPT_GLUT=-DGLUT_INCLUDE_DIR="%INSTALL_PATH%\include" -DGLUT_glut_LIBRARY_RELEASE="%INSTALL_PATH%\lib\freeglut.lib"

cd ..\..\..\..\..

echo ----- Building zlib -----

cd zlib

if not exist "build\" mkdir build
cd build

if exist "build_cvkit\" (
  cd build_cvkit\
) else (
  mkdir build_cvkit\
  cd build_cvkit\
  cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="%INSTALL_PATH%" ..\..
)

nmake install
if %ERRORLEVEL% NEQ 0 exit /b 1

cd ..\..\..

echo ----- Building libpng -----

cd libpng

if not exist "build\" mkdir build
cd build

if exist "build_cvkit\" (
  cd build_cvkit\
) else (
  mkdir build_cvkit\
  cd build_cvkit\
  cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release -DPNG_SHARED=OFF -DCMAKE_INSTALL_PREFIX="%INSTALL_PATH%" ..\..
)

nmake install
if %ERRORLEVEL% NEQ 0 exit /b 1

cd ..\..\..

echo ----- Building libjpeg -----

cd libjpeg

if not exist "build\" mkdir build
cd build

if exist "build_cvkit\" (
  cd build_cvkit\
) else (
  mkdir build_cvkit\
  cd build_cvkit\
  cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release -DENABLE_SHARED=OFF -DCMAKE_INSTALL_PREFIX="%INSTALL_PATH%" ..\..
)

nmake install
if %ERRORLEVEL% NEQ 0 exit /b 1

cd ..\..\..

echo ----- Building libtiff -----

cd libtiff

if not exist "build\" mkdir build
cd build

if exist "build_cvkit\" (
  cd build_cvkit\
) else (
  mkdir build_cvkit\
  cd build_cvkit\
  cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="%INSTALL_PATH%" -DBUILD_SHARED_LIBS=OFF -Dtiff-tools=OFF -Dtiff-tests=OFF -Dtiff-opengl=OFF -Dtiff-docs=OFF ..\.. 
)

nmake install
if %ERRORLEVEL% NEQ 0 exit /b 1

cd ..\..\..


echo ----- Building cvkit -----

cd cvkit\build

if exist "build_cvkit\" (
  cd build_cvkit\
) else (
  mkdir build_cvkit\
  cd build_cvkit\
  cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="%INSTALL_PATH%" %OPT_GLUT% %OPT_GLEW% ..\..
)

nmake install
if %ERRORLEVEL% NEQ 0 exit /b 1

cd ..


