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

#ifndef GMATH_CAMGEOM_H
#define GMATH_CAMGEOM_H

#include <gmath/linalg.h>

#include <stdlib.h>

namespace gmath
{

/**
   NOTE: In this module, R and T are used for transforming a point from the
   world into the camera coordinate system, i.e. Pc=R*Pw+T
*/

// --- Fundamental matrices operations ---

/**
   Computes the fundamental matrix from the camera matrices K0 and K1, i.e.
   [fx s u; 0 fy v; 0 0 1] and the essential matrix E. The fundamental matrix
   is unique up to a scale factor. See Hartley and Zisserman (2003, pp. 244).

   @param K0 Camera matrix, i.e. [fx s u; 0 fy v; 0 0 1]
   @param K1 Camera matrix, i.e. [fx s u; 0 fy v; 0 0 1]
   @param E  Essential matrix.
   @return   Fundamental matrix.
*/

inline Matrix33d createFundamental(const Matrix33d &K0, const Matrix33d &K1,
                                   const Matrix33d &E)
{
  return transpose(inv(K1))*E*inv(K0);
}

/**
   Computes the fundamental matrix from two projection matrices. The
   fundamental matrix is unique up to a scale factor. See Hartley and Zisserman
   (2003, pp. 244).

   @param P0 Projection matrix.
   @param P1 Projection matrix.
   @return   Fundamental matrix.
*/

inline Matrix33d createFundamental(const Matrix34d &P0, const Matrix34d &P1)
{
  Matrix33d U;
  Matrix44d V;
  Vector4d W;
  svd(P0, U, W, V);
  Vector3d e1=P1*V.getColumn(3);

  for (int i=0; i<W.size(); i++)
    if (W[i] != 0)
    {
      W[i]=1.0/W[i];
    }

  return createSkewSymmetric(e1)*P1*mul(V, W, U);
}

/**
   Normalizes a fundamental matrix so that the sum of squares of all elements
   becomes 1.

   @param F Fundamental matrix that will be changed.
*/

inline void normalizeFundamental(Matrix33d &F)
{
  double v=0;

  for (int k=0; k<F.rows(); k++)
  {
    for (int i=0; i<F.cols(); i++)
    {
      v+=F(k, i)*F(k, i);
    }
  }

  F/=sqrt(v);
}

/**
   Computes the location of epipole 0, which is the projection of the optical
   center of camera 1, in homogeneous coordinates, i.e. e0 such that F*e0=0.

   @param F Fundamental matrix.
   @return  Epipole 0 in homogeneous coordinates.
*/

inline Vector3d getEpipole0(const Matrix33d &F)
{
  Matrix33d U, V;
  Vector3d W;
  svd(F, U, W, V);

  return V.getColumn(2);
}

/**
   Computes the location of epipole 1, which is the projection of the optical
   center of camera 0 in homogeneous coordinates, i.e. e1 such that e1^T*F=0.

   @param F Fundamental matrix.
   @return  Epipole 1 in homogeneous coordinates.
*/

inline Vector3d getEpipole1(const Matrix33d &F)
{
  Matrix33d U, V;
  Vector3d W;
  svd(F, U, W, V);

  return U.getColumn(2);
}

/**
   The corresponding projections p0 and p1 are refined by the Sampson
   approximation, so that they approximately fullfill the epipolar constraint
   p1^T * F * p0=0. Returned is the Sampson error. This function is much
   faster than refineOptimal(). See Hartley and Zissermann, pp. 315.

   @param F  Fundamental matrix.
   @param p0 Point in camera 0 in homogeneous coordinates, which will be
             changed.
   @param p1 Point in camera 1 in homogeneous coordinates, which will be
             changed.
   @return   Sampson error before refinement.
*/

inline double refineSampson(const Matrix33d &F, Vector3d &p0, Vector3d &p1)
{
  p0/=p0[2];
  p1/=p1[2];

  double s0=F(0, 0)*p0[0]+F(0, 1)*p0[1]+F(0, 2);
  double s1=F(1, 0)*p0[0]+F(1, 1)*p0[1]+F(1, 2);
  double s2=F(0, 0)*p1[0]+F(1, 0)*p1[1]+F(2, 0);
  double s3=F(0, 1)*p1[0]+F(1, 1)*p1[1]+F(2, 1);
  double s=p1*F*p0/(s0*s0+s1*s1+s2*s2+s3*s3);

  p0[0]-=s*s2;
  p0[1]-=s*s3;
  p1[0]-=s*s0;
  p1[1]-=s*s1;

  return s;
}

/**
   The corresponding projections p0 and p1 are refined so that they exactly
   fullfill the epipolar constraint p1^T * F * p0=0. If the decomposition of
   F into camera matrices and rigid motion RT is known, then the function
   refineCorrespondence(RT, p0', p1') is faster.

   @param F  Fundamental matrix.
   @param p0 Point in camera 0 in homogeneous coordinates, which will be
             changed.
   @param p1 Point in camera 1 in homogeneous coordinates, which will be
             changed.
*/

void refineOptimal(const Matrix33d &F, Vector3d &p0, Vector3d &p1);

// --- Essential matrix operations ---

/**
   Constructs an essential matrix from a rotation and translation as defined
   by P1=R*P0+T. The essential matrix is unique up to a scale factor. See
   Hartley and Zisserman (2003, pp. 257).

   @param R Rotation matrix.
   @param T Translation vector.
   @return  Essential matrix.
*/

inline Matrix33d createEssential(const Matrix33d &R, const Vector3d &T)
{
  return createSkewSymmetric(T)*R;
}

/**
   Constructs an essential matrix from camera matrices K0 and K1 and a
   fundamental matrix.

   @param K0 Camera matrix, i.e. [fx s u; 0 fy v; 0 0 1]
   @param K1 Camera matrix, i.e. [fx s u; 0 fy v; 0 0 1]
   @param F  Fundamental matrix.
   @return   Essential matrix.
*/

inline Matrix33d createEssential(const Matrix33d &K0, const Matrix33d &K1,
                                 const Matrix33d &F)
{
  return transpose(K1)*F*K0;
}

/**
   Normalizes and ensures an essential matrix.

   @param E Essential matrix that will be changed.
*/

inline void normalizeEssential(Matrix33d &E)
{
  Matrix33d U, V;
  Vector3d W;
  svd(E, U, W, V);
  W[0]=1;
  W[1]=1;
  W[2]=0;
  E=mul(U, W, V);
}

// --- Projection matrix operations ---

/**
   Creation of a projection matrix from rotation and translation such that
   Pc=R*Pw+T. The projection matrix is unique up to a scale factor.

   @param R Rotation matrix.
   @param T Translation vector.
   @return  Projection matrix [R|T].
*/

inline Matrix34d createProjection(const Matrix33d &R,
                                  const Vector3d &T)
{
  Matrix34d P;

  for (int k=0; k<3; k++)
  {
    P(k, 0)=R(k, 0);
    P(k, 1)=R(k, 1);
    P(k, 2)=R(k, 2);
    P(k, 3)=T[k];
  }

  return P;
}

/**
   Creation of a projection matrix from a camera matrix and the rotation and
   translation such that Pc=R*Pw+T. The projection matrix is unique up to a
   scale factor.

   @param K Camera matrix, i.e. [fx s u; 0 fy v; 0 0 1]
   @param R Rotation matrix.
   @param T Translation vector.
   @return  Projection matrix K*[R|T].
*/

inline Matrix34d createProjection(const Matrix33d &K,
                                  const Matrix33d &R,
                                  const Vector3d &T)
{
  return K*createProjection(R, T);
}

/**
   Creation of a projection matrix for camera 1 from a fundamental matrix. The
   corresponding projection matrix for camera 0 is [I | 0]. The projection
   matrices are unique up to an arbitrary common projective transformation. See
   Hartley and Zisserman (2003, pp. 256, Result 9.14).

   @param F Fundamental matrix.
   @return  Projection matrix for camera 1.
*/

inline Matrix34d createProjection1(const Matrix33d &F)
{
  Vector3d e1=getEpipole1(F);
  Matrix33d A=createSkewSymmetric(e1)*F;
  Matrix34d P;

  for (int k=0; k<3; k++)
  {
    P(k, 0)=A(k, 0);
    P(k, 1)=A(k, 1);
    P(k, 2)=A(k, 2);
    P(k, 3)=e1[k];
  }

  return P;
}

/**
   Creation of a projection matrix for camera 1 from an essential matrix and
   point correspondences, which are used to resolve the inherent four fold
   ambiguity. The corresponding projection matrix for camera 0 is [I | 0] and
   for camera 1 it is [R|T]. See Hartley and Zisserman (2003, page 258)

   @param E  Essential matrix.
   @param p0 Projections in camera 0 in homogeneous coordinates.
   @param p1 Projections in camera 1 in homogeneous coordinates.
*/

Matrix34d createProjection1(const Matrix33d &E, const std::vector<Vector3d> &p0,
                            const std::vector<Vector3d> &p1);

/**
   Returns the rotation matrix from a projection matrix that contains only the
   rotation and translation.

   @param RT Projection matrix that consists only of a rotation and translation.
   @return   Rotation matrix.
*/

inline Matrix33d getRotation(const Matrix34d &RT)
{
  Matrix33d R;

  for (int k=0; k<3; k++)
    for (int i=0; i<3; i++)
    {
      R(k, i)=RT(k, i);
    }

  return R;
}

/**
   The corresponding projections p0 and p1 are refined so that they exactly
   fullfill the epipolar constraint exactly, i.e., the returned p0 and p1 are
   as close as possible to the given ones and p0=[I|0]*P and p1=RT1*P with
   RT1=[R|T]. This function is faster than the more general function that is
   based on the fundamental matrix.

   @param RT1 Projection matrix that consists only of a rotation and translation.
   @param p0  Projections in camera 0 in homogeneous coordinates, which are
              changed.
   @param p1  Projections in camera 1 in homogeneous coordinates, which are
              changed.
*/

void refineOptimal(const Matrix34d &RT1, Vector3d &p0, Vector3d &p1);

/**
   Projective reconstruction from image coordinates, by minimizing an algebraic
   error. The function refineOptimal() or refineSampson() should, but does not
   have to be called before. See Hartley and Zisserman (2003, pp. 312).

   @param P0 Projection matrix of camera 0.
   @param P1 Projection matrix of camera 1.
   @param p0 Projections in camera 0 in homogeneous coordinates.
   @param p1 Projections in camera 1 in homogeneous coordinates.
   @return   Reconstructed point in homogeneous coordinates. The point may be
             reconstructed behind the cameras.
*/

inline Vector4d reconstruct(const Matrix34d &P0, const Matrix34d &P1,
                            const Vector3d &p0, const Vector3d &p1)
{
  Matrix44d X;

  for (int i=0; i<4; i++)
  {
    X(0, i)=p0[0]/p0[2]*P0(2, i)-P0(0, i);
    X(1, i)=p0[1]/p0[2]*P0(2, i)-P0(1, i);
    X(2, i)=p1[0]/p1[2]*P1(2, i)-P1(0, i);
    X(3, i)=p1[1]/p1[2]*P1(2, i)-P1(1, i);
  }

  Matrix44d U, V;
  Vector4d W;
  svd(X, U, W, V);
  return V.getColumn(3);
}

/**
   Projective reconstruction from image coordinates, by minimizing an algebraic
   error. The function refineOptimal() or refineSampson() should, but does not
   have to be called before. See Hartley and Zisserman (2003, pp. 312).

   @param P0 Projection matrix of camera 0.
   @param P1 Projection matrix of camera 1.
   @param p0 Projections in camera 0 in inhomogeneous coordinates.
   @param p1 Projections in camera 1 in inhomogeneous coordinates.
   @return   Reconstructed point in homogeneous coordinates. The point may be
             reconstructed behind the cameras.
*/

inline Vector4d reconstruct(const Matrix34d &P0, const Matrix34d &P1,
                            const Vector2d &p0, const Vector2d &p1)
{
  Matrix44d X;

  for (int i=0; i<4; i++)
  {
    X(0, i)=p0[0]*P0(2, i)-P0(0, i);
    X(1, i)=p0[1]*P0(2, i)-P0(1, i);
    X(2, i)=p1[0]*P1(2, i)-P1(0, i);
    X(3, i)=p1[1]*P1(2, i)-P1(1, i);
  }

  Matrix44d U, V;
  Vector4d W;
  svd(X, U, W, V);
  return V.getColumn(3);
}

/**
   Reconstruction of a 3D point in the coordinate system of camera 0, from
   the correspondences p0 and p1 in homogeneous coordinates and the
   transformation of the coordinate system of camera 0 to 1, i.e.
   P1=R1*P0+T1.

   @param R1 Rotation matrix.
   @param T1 Translation vector.
   @param p0 Projections in camera 0 in homogeneous coordinates.
   @param p1 Projections in camera 1 in homogeneous coordinates.
   @param ff If true, then the reconstruction will be forced in front
             of the camera, otherwise it can also be behind the cameras.
   @return   Reconstructed point in inhomogeneous coordinates.
*/

inline Vector3d reconstruct(const Matrix33d &R1, const Vector3d &T1,
                            const Vector3d &p0, const Vector3d &p1, bool ff=false)
{
  /*
    The implementation is based on the distance between a point X0 and
    a line through X1 and X2, i.e. V=X1+t*(X2-X1). The closest point on
    the line is at parameter t=-(X1-X0)*(X2-X1)/|X2-X1|^2. Parameterizing
    the point X0 with the second line and doing this also the other way
    around leads to two equations with the two line parameters s and t
    as unknowns. If the reconstruction is in front both cameras, then
    both parameters must be positive.
  */

  Matrix33d Rt=transpose(R1);

  const Vector3d &A=p0;
  const Vector3d B=Rt*p1;
  const Vector3d C=Rt*-T1;

  double aa=A*A;
  double bb=B*B;
  double ab=A*B;
  double d=aa*bb-ab*ab;

  if (std::abs(d) < 1e-9)
  {
    return 1e12*norm(C)*(A+B);  // point near infinity relative to baseline
  }

  double ac=A*C;
  double bc=B*C;

  double s=(ac*bb-bc*ab)/d;
  double t=(ab*ac-bc*aa)/d;

  if (ff && (s <= 0 || t <= 0)) // ensure that point is reconstructed in front
  {
    return 1e12*norm(C)*(A+B);
  }

  return (s*A+t*B+C)/2;
}

/**
   Checks if a reconstruction of the corresponding point p0 and p1 is in front
   of both cameras, defined by the projection matrices [I|0] and [R1|T1].

   @param R1 Rotation matrix.
   @param T1 Translation vector.
   @param p0 Projections in camera 0 in homogeneous coordinates.
   @param p1 Projections in camera 1 in homogeneous coordinates.
   @return   true, if the point is in front of both cameras.
*/

inline bool reconstructInFront(const Matrix33d &R1, const Vector3d &T1,
                               const Vector3d &p0, const Vector3d &p1)
{
  Matrix33d Rt=transpose(R1);

  const Vector3d &A=p0;
  const Vector3d B=Rt*p1;
  const Vector3d C=Rt*-T1;

  double aa=A*A;
  double bb=B*B;
  double ab=A*B;
  double d=aa*bb-ab*ab;

  if (std::abs(d) < 1e-9)
  {
    if (ab > 0)
    {
      return true;
    }

    return false;
  }

  double ac=A*C;
  double bc=B*C;

  double s=(ac*bb-bc*ab)/d;
  double t=(ab*ac-bc*aa)/d;

  return t > 0 && s > 0;
}

/**
   This function computes the reprojection error of a point that is
   reconstructed by p0 and p1. The reprojection error in 4 dimensions is
   given for a focal length of 1. The cameras are defined by the projection
   matrices [I|0] and [R1|T1].

   @param R1   Rotation matrix.
   @param T1   Translation vector.
   @param E    Precalculated corresponding essential matrix, i.e.
               E=createEssential(R1, T1)
   @param p0   Projections in camera 0 in homogeneous coordinates.
   @param p1   Projections in camera 1 in homogeneous coordinates.
   @param best If true, then the correspondences are refined optimally,
               otherwise by Sampson approximation.
   @return     Reprojection error in both dimensions of both images.
*/

inline Vector4d reprojectionError(const Matrix33d &R1, const Vector3d &T1,
                                  const Matrix33d &E, const Vector3d &p0, const Vector3d &p1, bool best=false)
{
  Vector3d q0(p0);
  Vector3d q1(p1);

  if (best) // refine correspondences optimal or by Sampson approximation
  {
    refineOptimal(E, q0, q1);
  }
  else
  {
    refineSampson(E, q0, q1);
  }

  // the reconstruction is forced to be in front of the cameras

  Vector3d P=reconstruct(R1, T1, q0, q1, true);

  // computation of reprojection error

  Vector4d err(1e6, 1e6, 1e6, 1e6);

  if (P[2] > 0)
  {
    err[0]=P[0]/P[2]-p0[0]/p0[2];
    err[1]=P[1]/P[2]-p0[1]/p0[2];
  }

  P=R1*P+T1;

  if (P[2] > 0)
  {
    err[2]=P[0]/P[2]-p1[0]/p1[2];
    err[3]=P[1]/P[2]-p1[1]/p1[2];
  }

  return err;
}

}

#endif
