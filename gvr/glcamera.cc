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

#include "glcamera.h"

#include <gmath/linalg.h>
#include <cmath>

#include <iostream>

using std::min;

using gmath::Vector3d;
using gmath::Matrix33d;
using gmath::SMatrix;
using gmath::createR;

namespace gvr
{

/*
  OpenGL transformation to screen coordinates:
  
  | 2*f/w     0           0               0          |
  |   0    -2*f/h         0               0          |   | R^T  -R^T*T |   | Pw |
  |   0       0    (fz+nz)/(fz-nz)  -2*fz*nz/(fz-nz) | * |  0      1   | * |  1 |
  |   0       0           1               0          |
*/

void GLCamera::computeTransformation()
{
    SMatrix<double, 4, 4> P;
    
    P(0, 0)=2*f/width;
    P(1, 1)=-2*f/height;
    P(2, 2)=(fz+nz)/(fz-nz);
    P(2, 3)=-2*fz*nz/(fz-nz);
    P(3, 2)=1;
    P(3, 3)=0;
    
    SMatrix<double, 4, 4> RT;
    Vector3d V=-(transpose(R)*T);
    
    RT(0, 0)=R(0, 0); RT(0, 1)=R(1, 0); RT(0, 2)=R(2, 0); RT(0, 3)=V[0];
    RT(1, 0)=R(0, 1); RT(1, 1)=R(1, 1); RT(1, 2)=R(2, 1); RT(1, 3)=V[1];
    RT(2, 0)=R(0, 2); RT(2, 1)=R(1, 2); RT(2, 2)=R(2, 2); RT(2, 3)=V[2];
    
    P=P*RT;
    
      // convert to OpenGL
    
    int j=0;
    for (int k=0; k<4; k++)
    {
      for (int i=0; i<4; i++)
        trans[j++]=static_cast<GLfloat>(P(k, i));
    }
    
    gltrans[0]=trans[0]; gltrans[1]=trans[4]; gltrans[2]=trans[8]; gltrans[3]=trans[12];
    gltrans[4]=trans[1]; gltrans[5]=trans[5]; gltrans[6]=trans[9]; gltrans[7]=trans[13];
    gltrans[8]=trans[2]; gltrans[9]=trans[6]; gltrans[10]=trans[10]; gltrans[11]=trans[14];
    gltrans[12]=trans[3]; gltrans[13]=trans[7]; gltrans[14]=trans[11]; gltrans[15]=trans[15];
    
      // compute direction from which light is comming, e.g. viewpoint
    
    light[0]=static_cast<GLfloat>(-R(0, 2));
    light[1]=static_cast<GLfloat>(-R(1, 2));
    light[2]=static_cast<GLfloat>(-R(2, 2));
}

GLCamera::GLCamera()
{
    fov=50.0/180*M_PI;
    
    width=1;
    height=1;
    
    nz=0.1;
    fz=1;
    
    scale=0;
    texture=true;
    specialcolor=false;
    pointsonly=false;
    f=1;
    
    computeTransformation();
}

void GLCamera::init(const Vector3d &c, double size)
{
    R=0;
    
    R(0, 0)=1;
    R(1, 1)=-1;
    R(2, 2)=-1;
    
    T=c;
    T[2]+=size/(2*tan(fov/2));
    
    fz=10*size;
    if (fz <= 0)
      fz=1;
    
    nz=size/1000;
    
    computeTransformation();
    
    center=c;
}

void GLCamera::setSize(int w, int h)
{
    width=w;
    height=h;
    
    f=static_cast<float>(width/(2*tan(fov/2)));
    
    computeTransformation();
}

void GLCamera::setPose(const Matrix33d &_R, const Vector3d &_T)
{
    R=_R;
    T=_T;
    
    computeTransformation();
}

Vector3d GLCamera::pixel2World(int x, int y, float d)
{
    Vector3d P;
    
    P[2]=-2*fz*nz/(fz-nz)/(2*d-1-(fz+nz)/(fz-nz));
    P[0]=(x-width/2.0)/f*P[2];
    P[1]=(y-height/2.0)/f*P[2];
    
    return R*P+T;
}

bool GLCamera::onKey(unsigned char key, int x, int y)
{
    bool ret=false;
    
    switch (key)
    {
      case '+':
          if (scale == 0)
            scale=1.0;
          
          scale=1.5*scale;
          
          ret=true;
          break;
      
      case '-':
          if (scale == 0)
            scale=1.0;
          
          scale=scale/1.5;
          
          ret=true;
          break;
      
      case 't':
          if (!texture)
          {
            texture=true;
            specialcolor=false;
          }
          else
            texture=false;
          
          ret=true;
          break;
      
      case 's':
          if (!specialcolor)
          {
            specialcolor=true;
            texture=false;
          }
          else
            specialcolor=false;
          
          ret=true;
          break;
      
      case 'p':
          pointsonly=!pointsonly;
          ret=true;
          break;
      
      default:
          break;
    }
    
    return ret;
}


bool GLCamera::onMouseButton(int button, int state, int x, int y)
{
    bool ret=false;
    
    if (state == GLUT_DOWN)
    {
      mbutton=button;
      msx=x;
      msy=y;
      
        /* ensure pure rotation matrix and copy as start transformation */
      
      double ax, ay, az;
      recoverEuler(R, ax, ay, az, true);
      Rs=createR(ax, ay, az);
      Ts=T;
      
      if (button == 3)
      {
        mbutton=GLUT_RIGHT_BUTTON;
        ret=onMouseMove(x, y+40);
      }
      else if (button == 4)
      {
        mbutton=GLUT_RIGHT_BUTTON;
        ret=onMouseMove(x, y-40);
      }
      
        /* adapt step width */
      
      step=norm(T-center)/width;
    }
    
    return ret;
}

bool GLCamera::onMouseMove(int x, int y)
{
    bool ret=false;
    
    switch (mbutton)
    {
      case GLUT_LEFT_BUTTON:
          {
            Vector3d P1, P2;
            
            P1[0]=1.0-2.0*msx/width;
            P1[1]=1.0-2.0*msy/height;
            P1[2]=sqrt(1.0-min(1.0, P1[0]*P1[0]+P1[1]*P1[1]));
            P1/=norm(P1);
            
            P2[0]=1.0-2.0*x/width;
            P2[1]=1.0-2.0*y/height;
            P2[2]=sqrt(1.0-min(1.0, P2[0]*P2[0]+P2[1]*P2[1]));
            P2/=norm(P2);
            
            Matrix33d RR=createR(cross(P2, P1), 2.0*acos(min(1.0, P1*P2)));
            
            R=Rs*RR;
            T=center-Rs*RR*transpose(Rs)*(center-Ts);
            
            computeTransformation();
            ret=true;
          }
          break;
      
      case GLUT_MIDDLE_BUTTON:
          {
            Vector3d TT;
            
            TT[0]=-step*(x-msx);
            TT[1]=-step*(y-msy);
            TT[2]=0;
            
            T=Rs*TT+Ts;
            
            computeTransformation();
            ret=true;
          }
          break;
      
      case GLUT_RIGHT_BUTTON:
          {
            Vector3d TT;
            
            TT[0]=0;
            TT[1]=0;
            TT[2]=4*step*(y-msy);
            
            T=Rs*TT+Ts;
            
            computeTransformation();
            ret=true;
          }
          break;
    }
    
    return ret;
}

}
