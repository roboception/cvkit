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

#ifndef GVR_GLCAMERA_H
#define GVR_GLCAMERA_H

#include <gmath/smatrix.h>
#include <gmath/svector.h>

#include <GL/glew.h>

#ifdef __APPLE__
#include <glut.h>
#else
#include <GL/glut.h>
#endif

namespace gvr
{

class GLCamera
{
  private:

    double      fov;
    int         width, height;

    gmath::Matrix33d R;
    gmath::Vector3d  T;
    double           nz, fz;

    double      scale;
    bool        texture;
    bool        specialcolor;
    bool        pointsonly;
    double      f;
    GLfloat     trans[4*4];
    GLfloat     gltrans[4*4];
    GLfloat     light[3];

    int              mbutton, msx, msy;
    gmath::Vector3d  center;
    gmath::Matrix33d Rs;
    gmath::Vector3d  Ts;
    double           step;

    void computeTransformation();

  public:

    GLCamera();

    // initialise position, orientation and near and far plane of the camera
    // with center and size of scene

    void init(const gmath::Vector3d &c, double size);

    // size of window for rendering

    void setSize(int w, int h);

    int getWidth() { return width; }
    int getHeight() { return height; }

    // the focal length in pixel is f=w/(2*tan(field_of_view/2))

    double getFocalLength() const { return f; }

    // the transformation of a camera point to a world point is Pw=R*P+T

    const gmath::Matrix33d &getR() const { return R; }
    const gmath::Vector3d &getT() const { return T; }

    void setPose(const gmath::Matrix33d &R, const gmath::Vector3d &T);

    // convert from a pixel in the depth buffer into 3D

    gmath::Vector3d pixel2World(int x, int y, float d);

    // rendering settings like point scaling factor and color

    double getPointScale() const { return scale; }
    void setPointScale(double s) { scale=s; }

    bool getRenderTexture() const { return texture; }
    void setRenderTexture(bool t) { texture=t; }

    bool getRenderSpecialColor() const { return specialcolor; }
    void setRenderSpecialColor(bool sc) { specialcolor=sc; }

    bool getRenderPointsOnly() const { return pointsonly; }
    void setRenderPointsOnly(bool p) { pointsonly=p; }

    // returns the 4 x 4 transformation matrix and the transposed matrix
    // for OpenGL

    const GLfloat *getTransformation() const { return trans; }
    const GLfloat *getGLTransformation() const { return gltrans; }

    // returns a 3 dimensional that describes the illumination direction for
    // OpenGL

    const GLfloat *getGLLightDirection() const { return light; }

    // methods that handle events for changing the camera pose or kind of
    // rendering, the return value signals if the scene should be redrawn

    void setRotationCenter(const gmath::Vector3d &c) { center=c; }

    bool onKey(unsigned char key, int x, int y);
    bool onMouseMove(int x, int y);
    bool onMouseButton(int button, int state, int x, int y);
};

}

#endif