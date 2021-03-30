Version 2.3.0 (2021-03-30)
--------------------------

- Added optional disparity image parameters disp.scale, disp.offset and disp.inv

Version 2.2.1 (2020-11-25)
--------------------------

- Clean way to exit plyv using glutLeaveMainLoop instead of exit
- Added printing of mean intensity value in imgcmd
- Fix for Apples pthread implementation that does not know barriers

Version 2.2.0 (2020-08-21)
--------------------------

- Added parameter -recirpocal to imgcmd
- Fixing compiler warnings of gcc 9.3.0
- Loading corresponding param.txt and hdr file as image properties in ImageIO::loadProperties() method
- Building in gitlab without GDAL for arm bionic

Version 2.1.5 (2020-07-21)
--------------------------

- Fixed rounding when downscaling while loading images in PNG format

Version 2.1.4 (2020-07-20)
--------------------------

- Fixed building on gitlab

Version 2.1.3 (2020-07-20)
--------------------------

- Minimum requirement of GDAL is >= 2.0.0

Version 2.1.2 (2020-07-20)
--------------------------

- Loading downscaled images via GDAL using averaging instead of nearest
  neighbor as for the other formats too

Version 2.1.1 (2020-07-17)
--------------------------

- Making inverse distance weighted interpolation thread safe
- Added building for focal in gitlab file

Version 2.1.0 (2020-07-17)
--------------------------

- Added clip function in imgcmd
- Minimum requirement is now C++11
- Added class for creating thread barriers

Version 2.0.16 (2020-07-01)
---------------------------

- Added method for saving properties to ostream.

Version 2.0.15 (2020-06-02)
---------------------------

- Changed some methods of IDW class to const

Version 2.0.14 (2020-06-02)
---------------------------

- Added class for inverse distance weighted interpolation in a regular grid

Version 2.0.13 (2020-05-15)
---------------------------

- Fixed downscaling of float images with invalid values

Version 2.0.12 (2020-04-17)
---------------------------

- Added method to specify maximum value for visualizing a histogram

Version 2.0.11 (2020-04-03)
---------------------------

- new method for loading properties from std::istream
- use snprintf instead of printf for writing ply ASCII files

Version 2.0.10 (2020-02-07)
---------------------------

- Changed exception in Image template to standard runtime exception
- Fixed some compiler warnings

Version 2.0.9 (2019-12-19)
--------------------------

- Install header noise.h too

Version 2.0.8 (2019-12-19)
--------------------------

- New function for adding Gaussian noise to an image (available in imgcmd)
- Fixed potential problem with font height under Windows
- For text in Windows, use fixed font with black background as under Linux

Version 2.0.7 (2019-10-27)
--------------------------

- Low rating for file name containing 'right' for searching texture images in plyv and plycmd tool

Version 2.0.6 (2019-08-29)
--------------------------

- Added functions for converting pose to 4x4 transformation matrix and inverting transformation matrices
- Added method for specifying precision when adding values to a Property object
- Implement move constructor and move assignment operator for Image
- Fixes for compiling with Visual Studio
- Minor cmake changes

Version 2.0.5 (2019-07-03)
--------------------------

- Extended plane estimation class
- In plyv, fixed loading texture of Middlebury datasets
- Link bgui and gvr explicitely against libpng for resolving the
  png_get_copyright call when building on Mac
- Some minor fixes for building on Windows

Version 2.0.4 (2019-06-23)
--------------------------

- Fixed selecting negative values with 'b' or 'w' in sv tool
- Added possibility in imgcmd to paste images into different color channel

Version 2.0.3 (2019-04-07)
--------------------------

- Improved message queues

Version 2.0.2 (2019-03-29)
--------------------------

- Compile static libraries as relocatable

Version 2.0.1 (2019-02-26)
--------------------------

- Ignore outliers when computing extend of reconstructed scene for plyv
- Added rational and thin prism lens distortion models

Version 2.0.0 (2019-02-20)
--------------------------

- Added methods to remove properties related to camera distortion parameters

Version 1.9.15 (2019-01-24)
---------------------------

- In plyv, fixed size of camera if only one parameter file is loaded as camera
- plyv can now correctly map lower resolution texture to highe resolution depth image
- change README, etc. to markdown files
- Removed some asserts that are not necessary any more

Version 1.9.14 (2019-01-09)
---------------------------

- Added optional parameter for maximum disparity step for reconstruction of disparity images in plyv

Version 1.9.13 (2019-01-08)
---------------------------

- Added possibility to wrap existing image data with gimage::Image object
- Minor fixes for compiling with Visual Studio under Windows

Version 1.9.12 (2018-12-14)
---------------------------

- Added method for getting the replaced element from MsgQueueReplace::push()
- Added possibility to set info line and text from sub-class of GLWorld
- Added wrapper functions for redisplaying GL window and registration of GL timer function

Version 1.9.11 (2018-11-28)
---------------------------

- Minor fixes

Version 1.9.10 (2018-11-28)
---------------------------

- CI changes for building on Ubuntu 18.04

Version 1.9.9 (2018-10-26)
--------------------------

- Fixed loading texture image in plyv explicitly via 'i=' option

Version 1.9.8 (2018-10-16)
--------------------------

- Fixed wrong scaling of texture in some cases when plyv is called from sv via 'p' key
- Added heuristic for finding the correct texture image that corresponds to the disparity image

Version 1.9.7 (2018-10-15)
--------------------------

- Disabled syncFileByName() function under MinGW since fsync is not available

Version 1.9.6 (2018-09-14)
--------------------------

- Ensure that threads are joined in between if create is called more than once

Version 1.9.5 (2018-09-08)
--------------------------

- Fixed message queue

Version 1.9.4 (2018-08-22)
--------------------------

- Fixed missing reference in EstimatedPlane::getNormal() method
- Added missing delete in c++11 implementation of thread wrapper class

Version 1.9.3 (2018-03-20)
--------------------------

- Improved redrawing in bgui::BaseWindow::showBuffer() to avoid flickering of info text
  when changing image content

Version 1.9.2 (2018-02-28)
--------------------------

- Added function gutil::syncFileByName() for synchronizing files to their storage device
- plyv does not prefer color images over monochrome images for texture any more

Version 1.9.1 (2018-01-11)
--------------------------

- plyv now accepts texture images that are by an integer factor bigger than the
  corresponding disparity images
- Fixed gmath::recoverEuler() that failed for some rotation matrices

Version 1.9.0 (2017-12-28)
--------------------------

- Fixed some compile problems with MinGW64 and Visual Studio under Windows

Version 1.8.10 (2017-11-28)
---------------------------

- Fixed possible memory leakage in View::setCamera()

Version 1.8.9 (2017-07-03)
--------------------------

- Added drawing of polygons

Version 1.8.8 (2017-06-27)
--------------------------

- Fixed copy operator of PinholeCamera class

Version 1.8.7 (2017-06-07)
--------------------------

- Permit '/' for searching files under Windows
- Changed cmake so that dlls under windows are installed in bin instead of lib
- Fixed bug regarding reading of tiled images
- Changes and instructions for compilation under Visual Studio 2015

Version 1.8.6 (2017-05-10)
--------------------------

- Fixing some compile problems using MSYS2 / MinGW64 under Windows

Version 1.8.5 (2017-02-17)
--------------------------

- Changed ProcTime to measure relative time differences using the monotonic clock

Version 1.8.4 (2017-02-01)
--------------------------

- Fixed some minor issues that came up by cppcheck

Version 1.8.3 (2016-12-06)
--------------------------

- New helper class for computing a plane from sample points
- New helper function for skipping known std::strings in input streams
- Extended gimage::getNewImageName() so that it uses a given suffix
- Removed using declarations to avoid confusion
- Reformating all files

Version 1.8.2 (2016-11-07)
--------------------------

- Added helper class MsgQueue for inter-thread communication
- Added method to initialize all pixels of an image to a given value
- Changed cmake file for adding debian package number before distro codename

Version 1.8.0 (2016-11-02)
--------------------------

- Fixed several bugs when loading tiled images
- Added method for checking if an image contains valid pixels
- Speeded up image cropping function
- Added parameters for selecting the step width in minpack wrapper slmdif()
- Added functions for converting between errors and covariances
- Fixed bug when loading pinhole camera definition by id
- Added bicubic interpolation for accessing values in between pixels
- Improved speed for copying images
- gutil::Thread::getProcessingUnits() now returns number of configured instead
  of available CPUs
- Changed thread interface for more flexibility
- Fixed wrong index in gmath::getPose() from quaternion
- Added compare operator and getValue() method to gutil::Properties class
- Added possibility to specify intensity range in imageToJET() function
- Added function for fast rotating an image by 180 degrees
- Improved speed of downscaleImage() function
- Added functions for drawing arrows into gimage/paint.h
- Added explicit distortion models to gmath/camera
- Compiles as C++11 by default
- Some changes in cmake files

Version 1.7.0 (2015-10-26)
--------------------------

- Reduced requirement for OpenGL from 3.0 to 2.1 (i.e. GLSL 1.3 to 1.2).
  This permits successful compilation on Mac.
- The code for watching file changes via inotify has been made optional so that
  cvkit also compiles on systems that do support it, e.g. Mac.
- rainbow color mapping in sv is adapted to min/max intensity range, but will
  always be a power of 10.
- sv now has the additional parameter -scale for setting the initial image
  scale factor.
- imgcmd now has parameter -jet for converting disparity images into colored
  versions for visualization.
- Captured images in sv are now stored without a possible black border.
- Fixed bug when storing pgm, ppm or pfm under Windows.
- Some minor bugfixes.

Version 1.6.6 (2015-06-08)
--------------------------

- cmake config files will be installed together with libs for further
  development.

Version 1.6.2 - 1.6.5 (2015-05-18)
----------------------------------

- Removed "using namespace" declarations
- Minor changes and bugfixes

Version 1.6.1 (2015-02-16)
--------------------------

sv:
- Bugfix for 0.5 pixel shift error.

Version 1.6.0 (2014-12-19)
--------------------------

sv:
- If GDAL is compiled in, then the EXIF_Orientation tag that is given in jpeg
  images is used for automatic image rotation and flipping.
- 'f' flips the image horizontally.
- 'd' renames the current image by adding ".bak" and removes it from the list.
- 't' shows smoothed images that are computed by trilinear interpolation.
- Some internal changes.

Version 1.5.1 (2014-11-14)
--------------------------

- Internal cleanup.

Version 1.5.0 (2014-09-16)
--------------------------

- sv can now show individual color channels (see help).
- Header files and libs can optionally be installed for further development.
- Bugfix: Starting sv with a single image that does not exist caused showing
  all other images in that directory. An error is shown now.
- Internal cleanup.

Version 1.4.1 (2014-07-11)
--------------------------

- Minor bug fixes.

Version 1.4.0 (2014-06-16)
--------------------------

plyv:
- 's' toggles between shading in grey and special colors. Each object gets an
  individual color, which helps comparing data sets.
- 'b' toggles backface culling on (default) and off.
- <tab> toggle between color schemes.
- In the Middlebury naming convention, calib.txt is loaded automatically as
  camera definition file. If calib.txt is not specified explicitely, then the
  cameras are hidden by default and shown by pressing 'k'.
- Reduced requirement for OpenGL from 3.3 to 3.0 (i.e. GLSL 3.3 to 1.3).
- Some bugs fixed.

Version 1.3.0 (2014-06-06)
--------------------------

- Reduced requirement for cmake from 2.8.10 to 2.8.9.
- Documentation improved.
- Some bugs fixed.

Version 1.2.0 (2014-06-06)
--------------------------

- Reduced requirement for cmake from 2.8.12 to 2.8.10.
- Added -sub and -div options to imgcmd.

Version 1.1.0 (2014-06-05)
--------------------------

all:
- Added an installer for Windows.
- Optional support for reading and writing png in 8 and 16 bit using libpng.
- Makes sure that normals have length 1 when reading PLY files.
- Some bugs fixed.
- Documentation updated.

plyv:
- plyview has been renamed to plyv to avoid conflicts with other software
- Support for Middlebury naming convention and calib file format for on the fly
  3d visualization of ground truth datasets.
- If *.txt files are given, these files are loaded as parameters of cameras,
  that are visualized using pyramids.
- Captured images are now saved lossless in PNG format if libpng is compiled
  in. Otherwise, if GDAL is available, tif is used. As fallback, saving as ppm
  is used.

sv:
- If only one image is given as argument, then additionally to showing this
  image, all other images of the same directory can be selected by the left and
  right cursor keys.
- Showing the image name including path (if given) in the window title.
- Increased speed of zooming and panning using OpenMP.
- Captured images are saved as PNG by default, as in plyv (see above).

Version 1.0.0 (2014-05-05)
--------------------------

- First public release.
