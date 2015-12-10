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

#include "pose.h"

namespace gmath
{

namespace
{

/**
  Computes the parameter difference between two poses. This is not the same
  as the pose difference and thus not the inverse operation of combine!!!
*/

inline Vector6d diff(const Vector6d &p0, const Vector6d &p1)
{
  Vector6d ret;
  
  // difference of rotation parameters
  
  ret[3]=p0[3]-p1[3];
  ret[4]=p0[4]-p1[4];
  ret[5]=p0[5]-p1[5];
  
  // if the vectors point into different directions, try subtracting 2*pi
  
  if (p0[3]*p1[3]+p0[4]*p1[4]+p0[5]*p1[5] < 0)
  {
    double phi=sqrt(p0[3]*p0[3]+p0[4]*p0[4]+p0[5]*p0[5]);
    
    double a=p0[3]*(phi-2*pi)/phi;
    double b=p0[4]*(phi-2*pi)/phi;
    double c=p0[5]*(phi-2*pi)/phi;
    
    a-=p1[3];
    b-=p1[4];
    c-=p1[5];
    
    if (a*a+b*b+c*c < ret[3]*ret[3]+ret[4]*ret[4]+ret[5]*ret[5])
    {
      ret[3]=a;
      ret[4]=b;
      ret[5]=c;
    }
  }
  
  // difference of translation parameters
  
  ret[0]=p0[0]-p1[0];
  ret[1]=p0[1]-p1[1];
  ret[2]=p0[2]-p1[2];
  
  return ret;
}

}

Vector6d combineError(const Vector6d &pose0, const Vector6d &pose1,
                      const Vector6d &pose0_err, const Vector6d &pose1_err)
{
  Vector6d pose=combine(pose0, pose1);
  Vector6d perr;
  
  for (int i=0; i<6; i++)
  {
    Vector6d p=pose0;
    p[i]+=pose0_err[i];
    
    p=diff(combine(p, pose1), pose);
    
    for (int k=0; k<6; k++)
      perr[k]+=p[k]*p[k];
  }
  
  for (int i=0; i<6; i++)
  {
    Vector6d p=pose1;
    p[i]+=pose1_err[i];
    
    p=diff(combine(pose0, p), pose);
    
    for (int k=0; k<6; k++)
      perr[k]+=p[k]*p[k];
  }
  
  for (int i=0; i<6; i++)
    perr[i]=sqrt(perr[i]);
    
  return perr;
}

}
