/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2014, Institute of Robotics and Mechatronics, German Aerospace Center
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GIMAGE_VIEW_H
#define GIMAGE_VIEW_H

#include "image.h"

#include <gmath/camera.h>

#include <string>
#include <vector>

namespace gimage
{

/*
  The view class combines an image, the corresponding depth image as well as
  the corresponding camera. The view can be incomplete, e.g. images or the
  camera can be missing.
*/

class View
{
  private:
  
    ImageU8       image;
    ImageFloat    depth;
    gmath::Camera *camera;
    
  public:
  
    View();
    View(const View &v);
    ~View();
    
    const View &operator=(const View &v);
    
    void clear();
    
    void setCamera(const gmath::Camera *c);
    const gmath::Camera *getCamera() const { return camera; }
    
    void setImage(const ImageU8 &img);
    const ImageU8 &getImage() const { return image; }
    
    void setDepthImage(const ImageFloat &d);
    const ImageFloat &getDepthImage() const { return depth; }
};


/*
  Loads a view by the depth image name and optional appending specifications
  of the form:
  
  <depth file>[,p=<parameter file>][,i=<image file>][,ds=<s>][,x=<x>][,y=<y>][,w=<w>][,h=<h>]
  
  The parameter option can be given multiple times. If not given, then the
  method will try to find the parameters in the same directory or in the
  directories that are given in spath. If the image option is not given, then
  the method tries to find the image in the same directory or in the
  directories that are given in spath.
*/

void loadView(View &view, const char *name, const char *spath=0,
  bool verbose=false);

/*
  Fills the list with all possible alternatives for prefixes, derived from the
  given depth image name and search path.
*/

void getPrefixAlternatives(std::vector<std::string> &list, const std::string &depthname,
  const char *spath=0);

/*
  Loads properties from files that are specified in name or by searching.
*/

void loadViewProperties(gutil::Properties &prop, const char *name, const char *spath=0,
  bool verbose=false);

/*
  Extracts the image name from the given name or searches a suitable image.
*/

void getViewImageName(std::string &image, const char *name, const char *spath=0,
  bool verbose=false);
    
}

#endif
