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

#include "gltext.h"
#include "glcamera.h"

#include <iostream>

using std::string;

using gmath::Vector3d;
using gmath::Matrix33d;
using gmath::SMatrix;

namespace gvr
{

GLTextItem::GLTextItem(const string &t, const Matrix33d &R, const Vector3d &T,
  double s, bool center)
{
    text=t;
    
    s/=119.05;
    SMatrix<double, 4, 4> RT;
    RT(0, 0)=R(0, 0); RT(0, 1)=R(0, 1); RT(0, 2)=R(0, 2); RT(0, 3)=T[0];
    RT(1, 0)=R(1, 0); RT(1, 1)=R(1, 1); RT(1, 2)=R(1, 2); RT(1, 3)=T[1];
    RT(2, 0)=R(2, 0); RT(2, 1)=R(2, 1); RT(2, 2)=R(2, 2); RT(2, 3)=T[2];
    
    SMatrix<double, 4, 4> ms;
    ms(0, 0)=s;
    ms(1, 1)=-s;
    ms(2, 2)=-s;
    ms(3, 3)=1;
    
    if (center)
    {
      double len=0;
      for (size_t i=0; i<text.size(); i++)
        len+=glutStrokeWidth(GLUT_STROKE_ROMAN, text[i]);
      
      ms(0, 3)=-s*len/2;
    }
    
    ms(1, 3)=-s*33.3;
    
    RT=RT*ms;
    
    int j=0;
    for (int k=0; k<4; k++)
    {
      for (int i=0; i<4; i++)
        trans[j++]=static_cast<GLfloat>(RT(i, k));
    }
}

void GLText::addText(const string &text, const Matrix33d &R, const Vector3d &T,
  double scale, bool center)
{
    list.push_back(GLTextItem(text, R, T, scale, center));
}

void GLText::draw(const GLCamera &cam)
{
    glPushMatrix();
    glLoadIdentity();
    
    glColor3f(1.0, 1.0, 1.0);
    
    for (size_t i=0; i<list.size(); i++)
    {
      glLoadMatrixf(cam.getGLTransformation());
      glMultMatrixf(list[i].getGLTransformation());
      
      const char *p=list[i].getText().c_str();
      while (*p != '\0')
        glutStrokeCharacter(GLUT_STROKE_ROMAN, *p++);
    }
    
    glPopMatrix();
}

}
