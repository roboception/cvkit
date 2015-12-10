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

#include "camgeom.h"

#include <gmath/polynomial.h>
#include <gutil/exception.h>

#include <limits>

using std::numeric_limits;
using std::vector;

using gutil::InvalidArgumentException;

namespace gmath
{

void refineOptimal(const Matrix33d &F, Vector3d &p0, Vector3d &p1)
{
  // the implementation follows Hartley and Zisserman's (2003, pp. 318)
  
  Matrix33d invT0;
  Matrix33d invT1;
  invT0(0, 2)=p0[0]/p0[2];
  invT0(1, 2)=p0[1]/p0[2];
  invT1(0, 2)=p1[0]/p1[2];
  invT1(1, 2)=p1[1]/p1[2];
  
  // transform F
  
  Matrix33d F2=transpose(invT1)*F*invT0;
  
  // get epipoles of new fundamental matrix
  
  Matrix33d U, V;
  Vector3d W;
  svd(F2, U, W, V);
  Vector3d e0=V.getColumn(2);
  Vector3d e1=U.getColumn(2);
  
  // compute rotations
  
  e0/=sqrt(e0[0]*e0[0]+e0[1]*e0[1]);
  e1/=sqrt(e1[0]*e1[0]+e1[1]*e1[1]);
  Matrix33d R0;
  R0(0, 0)=e0[0];
  R0(0, 1)=e0[1];
  R0(1, 0)=-e0[1];
  R0(1, 1)=e0[0];
  R0(2, 2)=1;
  Matrix33d R1;
  R1(0, 0)=e1[0];
  R1(0, 1)=e1[1];
  R1(1, 0)=-e1[1];
  R1(1, 1)=e1[0];
  R1(2, 2)=1;
  
  // compute new F
  
  F2=R1*F2*transpose(R0);
  
  // compute coefficients of polynomial
  
  const double f=e0[2];
  const double g=e1[2];
  const double a=F2(1, 1);
  const double b=F2(1, 2);
  const double c=F2(2, 1);
  const double d=F2(2, 2);
  const double f2=f*f;
  const double f4=f2*f2;
  const double g2=g*g;
  const double a2=a*a;
  const double b2=b*b;
  const double c2=c*c;
  const double d2=d*d;
  
  Polynomiald p;
  p.set(0, b2*c*d-a*b*d2);
  p.set(1, b2*b2+b2*c2-a2*d2+2*b2*d2*g2+d2*d2*g2*g2);
  p.set(2, 4*a*b2*b+a*b*c2-a2*c*d+2*b2*c*d*f2-2*a*b*d2*f2+4*b2*c*d*g2+4*a*b*d2*g2
        +4*c*d2*d*g2*g2);
  p.set(3, 6*a2*b2+2*b2*c2*f2-2*a2*d2*f2+2*b2*c2*g2+8*a*b*c*d*g2+2*a2*d2*g2
        +6*c2*d2*g2*g2);
  p.set(4, 4*a2*a*b+2*a*b*c2*f2-2*a2*c*d*f2+b2*c*d*f4-a*b*d2*f4+4*a*b*c2*g2
        +4*a2*c*d*g2+4*c2*c*d*g2*g2);
  p.set(5, a2*a2+b2*c2*f4-a2*d2*f4+2*a2*c2*g2+c2*c2*g2*g2);
  p.set(6, a*b*c2*f4-a2*c*d*f4);
  
  // setting polynomial to 0 and get all real roots
  
  std::vector<double> r=realRoots(p);
  
  // find global minima (solution is t)
  
  double t=numeric_limits<double>::max();
  double mins=numeric_limits<double>::max();
  
  for (size_t i=0; i<r.size(); i++)
  {
    double s=(a*r[i]+b)*(a*r[i]+b)+g*g*(c*r[i]+d)*(c*r[i]+d);
    
    if (s != 0)
    {
      s=r[i]*r[i]/(1+f*f*r[i]*r[i])+(c*r[i]+d)*(c*r[i]+d)/s;
      
      if (s < mins)
      {
        t=r[i];
        mins=s;
      }
    }
  }
  
  // get optimal corner positions
  
  if (f*f > 0 && a*a+g*g*c*c > 0 && 1/(f*f)+c*c/(a*a+g*g*c*c) < mins)
  {
    p0[0]=f;
    p0[1]=0;
    p0[2]=f2;
    p1[0]=g*c2;
    p1[1]=-a*c;
    p1[2]=g2*c2+a2;
  }
  else
  {
    p0[0]=t*t*f;
    p0[1]=t;
    p0[2]=t*t*f2+1;
    p1[0]=g*(c*t+d)*(c*t+d);
    p1[1]=-(a*t+b)*(c*t+d);
    p1[2]=g2*(c*t+d)*(c*t+d)+(a*t+b)*(a*t+b);
  }
  
  // transform refined points back
  
  p0=invT0*transpose(R0)*p0;
  p1=invT1*transpose(R1)*p1;
}

Matrix34d createProjection1(const Matrix33d &E, const vector<Vector3d> &p0,
                            const vector<Vector3d> &p1)
{
  Matrix33d U, V;
  Vector3d W;
  svd(E, U, W, V);
  
  // ensure that det(R) will be 1 by inverting the column of U that belongs
  // to zero singular value
  
  if (det(U) < 0)
    U.setColumn(2, -U.getColumn(2));
    
  if (det(V) < 0)
    V.setColumn(2, -V.getColumn(2));
    
  // create all four cases of projection matrices
  
  Matrix33d S;
  
  S(0, 0)=0;
  S(0, 1)=-1;
  S(1, 0)=1;
  S(1, 1)=0;
  S(2, 2)=1;
  
  Matrix33d R1[4];
  Vector3d  T1[4];
  
  R1[0]=U*S*transpose(V);
  T1[0]=U.getColumn(2);
  R1[1]=R1[0];
  T1[1]=-T1[0];
  R1[2]=U*transpose(S)*transpose(V);
  T1[2]=T1[0];
  R1[3]=R1[2];
  T1[3]=-T1[2];
  
  // determine the solution that is consistent with all corresponding
  // points
  
  int j=-1;
  
  for (int i=0; i<4; i++)
  {
    bool ok=true;
    
    for (size_t k=0; k<p0.size() && ok; k++)
      ok=reconstructInFront(R1[i], T1[i], p0[k], p1[k]);
      
    if (ok)
      j=i;
  }
  
  if (j < 0)
    throw InvalidArgumentException("Cannot recover transformation from essential matrix");
    
  return createProjection(R1[j], T1[j]);
}

void refineOptimal(const Matrix34d &RT1, Vector3d &p0, Vector3d &p1)
{
  const Matrix33d R=getRotation(RT1);
  const Vector3d T=RT1.getColumn(3);
  Matrix33d F=createEssential(R, T);
  
  // the implementation follows Hartley and Zisserman's (2003, pp. 318)
  
  Matrix33d invT0;
  Matrix33d invT1;
  invT0(0, 2)=p0[0]/p0[2];
  invT0(1, 2)=p0[1]/p0[2];
  invT1(0, 2)=p1[0]/p1[2];
  invT1(1, 2)=p1[1]/p1[2];
  
  // transform F
  
  F=transpose(invT1)*F*invT0;
  
  // get epipoles that correspond to new fundamental matrix
  
  Vector3d e1=T;
  Vector3d e0=transpose(R)*-T;
  
  e0/=e0[2];
  e1/=e1[2];
  e0[0]-=p0[0]/p0[2];
  e0[1]-=p0[1]/p0[2];
  e1[0]-=p1[0]/p1[2];
  e1[1]-=p1[1]/p1[2];
  
  // compute rotations
  
  e0/=sqrt(e0[0]*e0[0]+e0[1]*e0[1]);
  e1/=sqrt(e1[0]*e1[0]+e1[1]*e1[1]);
  
  Matrix33d R0;
  R0(0, 0)=e0[0];
  R0(0, 1)=e0[1];
  R0(1, 0)=-e0[1];
  R0(1, 1)=e0[0];
  R0(2, 2)=1;
  
  Matrix33d R1;
  R1(0, 0)=e1[0];
  R1(0, 1)=e1[1];
  R1(1, 0)=-e1[1];
  R1(1, 1)=e1[0];
  R1(2, 2)=1;
  
  // compute new F
  
  F=R1*F*transpose(R0);
  
  // compute coefficients of polynomial
  
  const double f=e0[2];
  const double g=e1[2];
  const double a=F(1, 1);
  const double b=F(1, 2);
  const double c=F(2, 1);
  const double d=F(2, 2);
  const double f2=f*f;
  const double f4=f2*f2;
  const double g2=g*g;
  const double a2=a*a;
  const double b2=b*b;
  const double c2=c*c;
  const double d2=d*d;
  
  Polynomiald p;
  p.set(0, b2*c*d-a*b*d2);
  p.set(1, b2*b2+b2*c2-a2*d2+2*b2*d2*g2+d2*d2*g2*g2);
  p.set(2, 4*a*b2*b+a*b*c2-a2*c*d+2*b2*c*d*f2-2*a*b*d2*f2+4*b2*c*d*g2+4*a*b*d2*g2
        +4*c*d2*d*g2*g2);
  p.set(3, 6*a2*b2+2*b2*c2*f2-2*a2*d2*f2+2*b2*c2*g2+8*a*b*c*d*g2+2*a2*d2*g2
        +6*c2*d2*g2*g2);
  p.set(4, 4*a2*a*b+2*a*b*c2*f2-2*a2*c*d*f2+b2*c*d*f4-a*b*d2*f4+4*a*b*c2*g2
        +4*a2*c*d*g2+4*c2*c*d*g2*g2);
  p.set(5, a2*a2+b2*c2*f4-a2*d2*f4+2*a2*c2*g2+c2*c2*g2*g2);
  p.set(6, a*b*c2*f4-a2*c*d*f4);
  
  // setting polynomial to 0 and get all real roots
  
  vector<double> r=realRoots(p);
  
  // find global minima (solution is t)
  
  double t=numeric_limits<double>::max();
  double mins=numeric_limits<double>::max();
  
  for (size_t i=0; i<r.size(); i++)
  {
    double s=(a*r[i]+b)*(a*r[i]+b)+g*g*(c*r[i]+d)*(c*r[i]+d);
    
    if (s != 0)
    {
      s=r[i]*r[i]/(1+f*f*r[i]*r[i])+(c*r[i]+d)*(c*r[i]+d)/s;
      
      if (s < mins)
      {
        t=r[i];
        mins=s;
      }
    }
  }
  
  // get optimal corner positions
  
  if (f*f > 0 && a*a+g*g*c*c > 0 && 1/(f*f)+c*c/(a*a+g*g*c*c) < mins)
  {
    p0[0]=f;
    p0[1]=0;
    p0[2]=f2;
    p1[0]=g*c2;
    p1[1]=-a*c;
    p1[2]=g2*c2+a2;
  }
  else
  {
    p0[0]=t*t*f;
    p0[1]=t;
    p0[2]=t*t*f2+1;
    p1[0]=g*(c*t+d)*(c*t+d);
    p1[1]=-(a*t+b)*(c*t+d);
    p1[2]=g2*(c*t+d)*(c*t+d)+(a*t+b)*(a*t+b);
  }
  
  // transform refined points back
  
  p0=invT0*transpose(R0)*p0;
  p1=invT1*transpose(R1)*p1;
}

}
