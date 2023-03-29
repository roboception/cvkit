
INSTALLING CVKIT
================

Minimum requirements:

- Linux with kernel version >= 2.6.27 or Windows >= XP
- cmake >= 3.1.0
- g++ compiler (>= 4.1.2, although older versions might work)
- X11 or Windows for building sv.
- OpenGL 2.1, GLEW and GLUT (or FLTK) for building plyv.

Optional packages:

- libjpeg for loading and saving JPG images with 8 bits per color.
- libpng for loading and saving PNG files with 8 or 16 bits per color.
- libtiff for loading and saving TIFF images with 8 or 16 bit integer
  as well as 32 bit floating point values as data.
- GDAL (www.gdal.org) version >= 2.0 is used for loading many different
  scientific raster formats (including JPEG, PNG and TIFF).

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

For installing you may change the `CMAKE_INSTALL_PREFIX` using
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

Compiling under Windows
-----------------------

There is a build_win.bat script that can be executed in the command prompt of
Visual Studio. It was developed with version 2019. The script checks for the
availability of git and cmake and suggests web pages for downloading if they
cannot be found. Then, it automatically downloads dependencies like freeglut,
glew, zlib, libpng, libjpeg, and libtiff, compiles everything and creates a
self installable package with NSIS, which must be available.

