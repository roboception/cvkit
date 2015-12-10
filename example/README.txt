
CVKIT EXAMPLES
--------------

This folder contains some example files. The images can be visualized with:

> sv 1097_disp.pfm 1097_rgb.ppm

'h' will show all available key codes. The floating point depth image can be
color coded by pressing the 'm' key. 'p' will show a 3D reconstruction of the
currently visible part of the depth image. The rotation center is set to the
middle of the reconstructed scene part by default. It is best to double click
onto the church to reset the rotation center, before rotating.

When calling plyv directly, the cameras can be visualized as well with:

> plyv 1097_disp.pfm *param.txt

For creating a 3D reconstruction and storing it in PLY format, call:

> plycmd 1097_disp.pfm -out 1097.ply

Special formats and the structure of parameter files are explained in detail in
USAGE.txt.
