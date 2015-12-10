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

#ifndef GVR_GLWORLD_H
#define GVR_GLWORLD_H

#include "glmain.h"
#include "glcamera.h"

#include <gmath/svector.h>
#include <gutil/proctime.h>

#include <vector>

/**
  This is the world that contains a list of GLObject that know how to render
  themselves using OpenGL and GLCamera that describes the camera that sees the
  scene. The callback functions are overloaded in order to handle all events
  for drawing, moving, etc using OpenGL and GLUT functions.
*/

namespace gvr
{

class Model;
class GLObject;

class GLWorld : public GLListener
{
  private:
    
    std::string       prefix;
    
    gmath::Vector3d   offset;
    std::vector<GLObject*> list;
    gmath::Vector3d   extmin, extmax;
    gmath::Matrix33d  defRc;
    gmath::Vector3d   defTc;
    
    std::vector<bool> showid;
    GLCamera          camera;
    bool              fullscreen;
    int               colorschema;
    long              txt_rgb;
    
    gutil::ProcTime   mt;
    int               mb, mx, my;
    int               mod;
    
    std::string       infotext;
    std::string       infoline;
    
    GLWorld(const GLWorld &);
    GLWorld& operator=(const GLWorld &);
    
  public:
    
    GLWorld(int w, int h);
    virtual ~GLWorld();
    
    void setCapturePrefix(const char *p) { prefix=std::string(p); }
    
    const gmath::Vector3d &getOffset() const { return offset; }
    void setOffset(const gmath::Vector3d v) { offset=v; }
    
    void addModel(Model &model);
    void removeAllModels(int id);
    void showCameras(bool show);
    
    void resetCamera();
    
    void onRedraw();
    void onReshape(int w, int h);
    void onKey(unsigned char key, int x, int y);
    void onMouseButton(int button, int state, int x, int y);
    void onMouseMove(int x, int y);
};

}

#endif
