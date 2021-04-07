/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2021, Institute of Robotics and Mechatronics, German Aerospace Center
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

#ifndef GMATH_QUATERNION_H
#define GMATH_QUATERNION_H

#include "svector.h"
#include <cmath>

namespace gmath
{

/**
  Definition of quaternion i*x + j*y + k*z + w as Quaternion(x, y, z, w)
*/

typedef SVector<double, 4> Quaternion;

/**
  Returns the conjugated quaternion.
*/

inline Quaternion conj(const Quaternion &q)
{
  return Quaternion(-q[0], -q[1], -q[2], q[3]);
}

/**
  Returns the Grassmann product of two quaternions.
*/

inline Quaternion mul(const Quaternion &a, const Quaternion &b)
{
  return Quaternion(a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
                    a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
                    a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
                    a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]);
}

/**
  Constructs a quaternion for rotating around x-axis.
*/

inline Quaternion createQx(double a)
{
  a*=0.5;
  return Quaternion(sin(a), 0, 0, cos(a));
}

/**
  Constructs a quaternion for rotating around y-axis.
*/

inline Quaternion createQy(double a)
{
  a*=0.5;
  return Quaternion(0, sin(a), 0, cos(a));
}

/**
  Constructs a quaternion for rotating around z-axis.
*/

inline Quaternion createQz(double a)
{
  a*=0.5;
  return Quaternion(0, 0, sin(a), cos(a));
}

/**
  Rotates the given 3D point using a quaternion. The quaternion must have norm 1.
*/

inline Vector3d rot(const Quaternion &q, const Vector3d &p)
{
  Vector4d r=mul(mul(q, Vector4d(p[0], p[1], p[2], 0)), conj(q));

  return Vector3d(r[0], r[1], r[2]);
}

/**
  Conversion of a rotation quaternion (which must have norm 1) into a rotation
  matrix.
*/

inline Matrix33d getRotation(const Quaternion &q)
{
  Matrix33d M;

  M(0, 0) = 0.5 - q[1]*q[1] - q[2]*q[2];
  M(0, 1) = q[0]*q[1] - q[2]*q[3];
  M(0, 2) = q[0]*q[2] + q[1]*q[3];
  M(1, 0) = q[0]*q[1] + q[2]*q[3];
  M(1, 1) = 0.5 - q[0]*q[0] - q[2]*q[2];
  M(1, 2) = q[1]*q[2] - q[0]*q[3];
  M(2, 0) = q[0]*q[2] - q[1]*q[3];
  M(2, 1) = q[1]*q[2] + q[0]*q[3];
  M(2, 2) = 0.5 - q[0]*q[0] - q[1]*q[1];

  return 2*M;
}

/**
  Conversion of a rotation matrix (which must have det 1) into a quaternion.
*/

inline Quaternion getQuaternion(const Matrix33d &r)
{
  Quaternion q;

  q[0] = sqrt(std::max(0.0, 1 + r(0, 0) - r(1, 1) - r(2, 2)))/2;
  q[1] = sqrt(std::max(0.0, 1 - r(0, 0) + r(1, 1) - r(2, 2)))/2;
  q[2] = sqrt(std::max(0.0, 1 - r(0, 0) - r(1, 1) + r(2, 2)))/2;
  q[3] = sqrt(std::max(0.0, 1 + r(0, 0) + r(1, 1) + r(2, 2)))/2;

  if (r(2, 1) < r(1, 2)) q[0]=-q[0];
  if (r(0, 2) < r(2, 0)) q[1]=-q[1];
  if (r(1, 0) < r(0, 1)) q[2]=-q[2];

  return q;
}

}

#endif
