Computer Vision Toolkit (cvkit)
-------------------------------

cvkit is available for Linux as well as for Windows. It offers useful tools
for viewing and analyzing images and 3d models.

sv is a simple / scientific image viewer that can display monochrome and color
images with 8 and 16 bit integer as well as 32 bit float values as data types
per color channel. Functions include showing monochrome images with color
encoding, defining radiometric ranges, zooming and automatically reloading
images (Linux only). For image comparison, settings like zoom, radiometric
range, etc, can be kept while switching between images. Depth images (full or
parts) with associated camera parameter files can be visualized on the fly in
3D. sv natively supports the PGM, PPM and PFM image formats as well as TIFF
with 8 and 16 bit integer and 32 bit float values. TIFF, JPG, PNG, GIF and many
other raster data formats are supported through optional libraries like GDAL.

plyv is a simple but pretty fast viewer for colored point clouds and meshes
with per vertex coloring, shading and texture images. It also supports
on-the-fly conversion and visualization of depth images and cameras. plyv is
based on OpenGL and can cope with big data sets that consist of many million
vertices and triangles. Mainly the PLY format is supported, which has been
invented at Stanford University as an extendable format for storing vertices
and polygons together with additional information. It is especially useful for
scanned real-world data.

See [INSTALL.md](INSTALL.md) and [USAGE.md](USAGE.md) for more information.

Acknowledgments
---------------

The initial version of cvkit has been developed while I was working for the
Institute of Robotics and Mechatronics of the German Aerospace Center (DLR).
It has been published under the BSD license for supporting version 3 of the
Middlebury stereo evaluation.  I am now with the Roboception GmbH, which is
a DLR spin-off company.

I would like to thank Daniel Scharstein for testing the tools, giving me
feedback and motivating me to make the tools publicly available.  Thanks to
Daniel Scharstein also for the code for the jet and rainbow color coding as
well as testing it on Mac.

This software includes software developed by the University of Chicago, as
Operator of Argonne National Laboratory.

Contact
-------

    Dr. Heiko Hirschmueller
    Roboception GmbH
    Kaflerstrasse 2
    81241 Muenchen
    Germany

Email: heiko.hirschmueller@roboception.de
