/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2016 Roboception GmbH
 * Copyright (c) 2014 Institute of Robotics and Mechatronics, German Aerospace Center
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

#ifndef GMATH_POSE_H
#define GMATH_POSE_H

#include "camgeom.h"

#include <gmath/linalg.h>

#include <vector>

namespace gmath
{

/**
   NOTE: In this module, R and T (and the pose vector) are used for
   transforming a point from the camera into the world coordinate system, i.e.
   Pw=R*Pc+T
*/

/**
   Conversion between pose, rotation matrix and translation vector.
*/

inline Matrix33d getRotation(const Vector6d &pose)
{
  Vector3d n(pose[3], pose[4], pose[5]);
  double phi=norm(n);

  if (phi != 0)
    return createR(n/phi, phi);

  return Matrix33d();
}

inline Vector3d getTranslation(const Vector6d &pose)
{
  return Vector3d(pose[0], pose[1], pose[2]);
}

inline Vector6d getPose(const Matrix33d &R, const Vector3d &T)
{
  Vector3d n;
  double   phi=recoverAngleAxis(R, n);
  Vector6d pose(T[0], T[1], T[2], 0, 0, 0);

  if (phi != 0)
  {
    pose[3]=n[0]*phi;
    pose[4]=n[1]*phi;
    pose[5]=n[2]*phi;
  }

  return pose;
}

inline Vector6d getPose(const Matrix34d &RT)
{
  Vector3d n;
  double   phi=recoverAngleAxis(getRotation(RT), n);
  Vector6d pose(RT(0, 3), RT(1, 3), RT(2, 3), 0, 0, 0);

  if (phi != 0)
  {
    pose[3]=n[0]*phi;
    pose[4]=n[1]*phi;
    pose[5]=n[2]*phi;
  }

  return pose;
}

inline Vector4d getQuaternion(const Vector6d &pose)
{
  Vector3d n(pose[3], pose[4], pose[5]);
  double phi=norm(n);
  double s=std::sin(phi/2);

  if (phi != 0)
    s/=phi;

  return Vector4d(n[0]*s, n[1]*s, n[2]*s, std::cos(phi/2));
}

inline Vector6d getPose(const Vector4d &Q, const Vector3d &T)
{
  double phi=2*std::acos(Q[3]);
  Vector6d pose(T[0], T[1], T[2], 0, 0, 0);

  if (phi != 0 && Q[3] != 1)
  {
    double s=std::sqrt(1-Q[3]*Q[3]);
    pose[3]=phi*Q[0]/s;
    pose[4]=phi*Q[1]/s;
    pose[5]=phi*Q[2]/s;
  }

  return pose;
}

/**
   Compute the combination of two pose transformations.

   @param pose0 Pose 0.
   @param pose1 Pose relative to pose 0.
   @return      Combination of pose 0 and pose 1.
*/

inline Vector6d combine(const Vector6d &pose0, const Vector6d &pose1)
{
  Matrix33d R=getRotation(pose0);
  Vector3d  T=R*getTranslation(pose1)+getTranslation(pose0);
  R=R*getRotation(pose1);
  return getPose(R, T);
}

/**
   Returns the errors (i.e. standard deviations) of the parameters from a
   covariance matrix.

   @param pose_cov Covarience matrix.
   @return         Error vector.
*/

inline Vector6d getError(const Matrix66d &pose_cov)
{
  Vector6d ret;

  for (int i=0; i<6; i++)
  {
    ret[i]=std::sqrt(pose_cov(i, i));
  }

  return ret;
}

/**
   Returns a covariance matrix from an error vector with standard deviations.

   @param pose_err Error vector.
   @return         Covarience matrix.
*/

inline Matrix66d getCovariance(const Vector6d pose_err)
{
  Matrix66d ret;

  for (int i=0; i<6; i++)
  {
    ret(i, i)=pose_err[i]*pose_err[i];
  }

  return ret;
}

/**
   Combines the pose errors according to the combination of two pose
   transformations.

   @param pose0     Pose 0.
   @param pose1     Pose relative to pose 0.
   @param pose0_err Error of pose 0.
   @param pose1_err Error of pose 1.
   @return          Combination of pose error 0 and 1.
*/

Vector6d combineError(const Vector6d &pose0, const Vector6d &pose1,
  const Vector6d &pose0_err, const Vector6d &pose1_err);

}

#endif