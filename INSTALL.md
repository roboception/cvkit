
INSTALLING CVKIT
----------------

Minimum requirements:

- Linux with kernel version >= 2.6.27 or Windows >= XP
- cmake (>= 2.8.9, although older version might work if you change the minimum
  requirement at the top of the main CMakeLists.txt file)
- g++ compiler (>= 4.1.2, although older versions might work)
- X11 or Windows for building sv.
- OpenGL 2.1, GLEW and GLUT for building plyv.

Optional packages:

- libjpeg for loading and saving JPG images with 8 bits per color.
- libpng for loading and saving PNG files with 8 or 16 bits per color.
- GDAL (www.gdal.org) is used for loading many different scientific raster
  formats. It also supports loading and saving TIFF images with 8 or 16 bit
  integer or 32 bit floating point values.

If none of these libraries are available, only PGM, PPM and PFM are
supported for loading and saving images.

Compiling and installing under Linux
------------------------------------

For compiling cvkit, just extract the source package, go to the main
directory and type:

mkdir build
cd build
cmake ..
# (optionally 'ccmake ..' if you want to change some build options)
make

For installing you may change the CMAKE_INSTALL_PREFIX using 
ccmake and then type:

make install

Installing precompiled Windows packages
---------------------------------------

There is a windows package with pre-compiled binaries and an installer. Since
sv.exe and plyv.exe only have a simple GUI, images and PLY files must be
provided as parameters when starting the tools. The installer will
automatically add file associations, such that a double click to a PLY file
will start plyv.exe with that file as parameter. Similarly a double click to
PFM, PGM, PPM, TIF, JPG, PNG or VRT file will automatically open it with
sv.exe.

For opening more than one file at once, a link to plyv.exe and sv.exe can be
manually created on the desktop. Selecting several files, dragging and dropping
them on the icon will start plyv.exe or sv.exe with all those files.

Compiling under Windows using MinGW64 / MSYS2 
---------------------------------------------

Install MSYS2 (64) from http://www.msys2.org/, e.g.
http://repo.msys2.org/distrib/x86_64/msys2-x86_64-20161025.exe

Add the link 'MSYS2 MinGW 64 bit' to the desktop and start it to open a bash
shell. First, update all packages:

<code>pacman -Syu</code>

Usually this has to be done twice. Then install additional packages (but do
not use the MSYS version of gcc, but only mingw-w64-...), e.g.:

<code>pacman -S git nano make tar gzip zip mingw-w64-x86_64-cmake 
mingw-w64-x86_64-gcc mingw-w64-x86_64-freeglut mingw-w64-x86_64-glew 
mingw-w64-x86_64-crypto++ mingw-w64-x86_64-eigen3</code>

Optionally you may want to use GDAL. Download GDAL from 
http://trac.osgeo.org/gdal/wiki/DownloadSource, 
e.g. http://download.osgeo.org/gdal/2.2.0/gdal-2.2.0.tar.gz. Unpack and 
change into the main directory. Then type:

./configure
make
make install

Building and installing of cvkit works as under Linux. Go to the main 
directory and type:

mkdir build
cd build
cmake -G"MSYS Makefiles" ..
# (optionally you may edit the file CMakeCache.txt for changing some 
# variables, like CMAKE_INSTALL_PREFIX)
make
make install

Compiling under Windows using Visual Studio
-------------------------------------------

It is assumed that Visual Studio 2015 (Version 14) is installed. Other
versions may work as well.

For compiling plyv, you need to install GLEW and GLUT as well as OpenGL. 
This has not yet been tested with Visual Studio.

Optionally you may want to install GDAL. Download GDAL from 
http://trac.osgeo.org/gdal/wiki/DownloadSource, 
e.g. http://download.osgeo.org/gdal/2.2.0/gdal220.zip

Extract the archive. Open PowerShell (or another shell) and go to the main
directory of the unpacked archive. Type:

./generate_vcxproj.bat 14.0 64 gdal_vs2015

Use different parameters if you have are using a different version of 
Visual Studio. Double click on the generated gdal_vs2015 project file
for opening it in Visual Studio. Select 'Release' configuration in the
toolbar. Compile the project by pressing 'F7'. Close Visual Studio 
after compilation ends.

We will create an installation directory on the same level as the main 
directories of the GDAL and cvkit source. The directory will be called
'install'. Open PowerShell (or another shell) and go into the main 
directory of GDAL:

mkdir ..\install\bin
cp .\apps\*.exe ..\install\bin
cp .\gdal*.dll ..\install\bin
mkdir ..\install\include\gdal
cp *\*.h ..\install\include\gdal
mkdir ..\install\lib
cp .\gdal_i.lib ..\install\lib\

Install cmake for Windows from https://cmake.org/download/, e.g.
https://cmake.org/files/v3.8/cmake-3.8.1-win64-x64.msi.

- Start cmake-gui
- Select the cvkit source directory as source code path
- Copy the source code path to build the binaries and add /build to the 
  path
- Click 'Configure', confirm creating the build directory
- Select the appropriate Visual Studio version (e.g. 'Visual Studio 14 
  2015 Win64'), select 'Use default native compilers' and click 'Finish'
- Search for the key name 'CMAKE_INSTALL_PREFIX' and change the value to 
  the install path for GDAL
- Search for the key name 'USE_GDAL' and make sure that it is selected
- Click 'Configure' again to enable cmake to find GDAL.
- Click 'Generate'
- Click 'Open Project'
- In Visual Studio, select 'Release' in the toolbar
- Compile by pressing 'F7'
- In the project view on the left, select INSTALL, open the context menu
  with the right mouse button and select create

All binaries can be found in install/bin and may be copied to any other 
place. sv.exe can be selected as standard application for images so that
a double click on the images opens sv automatically.
