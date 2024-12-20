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

#ifndef GMATH_LINALG_H
#define GMATH_LINALG_H

#include "svector.h"
#include "smatrix.h"

#include "dvector.h"
#include "dmatrix.h"

#include <gutil/exception.h>

#include <algorithm>

namespace gmath
{

// --- Constants ---

const double pi=3.14159265358979323846;

// --- Functions for 3x3 rotation matrices of type double ---

Matrix33d ensureRotation(const Matrix33d &R);

Matrix33d createRx(double a);
Matrix33d createRy(double a);
Matrix33d createRz(double a);

/**
 * R(ax, ay, az)=Rx(ax)*Ry(-ay)*Rz(az)
 */

Matrix33d createR(double ax, double ay, double az);
bool recoverEuler(const Matrix33d &R, double &ax, double &ay, double &az, bool tolerant=false);

/**
 * Conversion between rotation matrix and rotation in angle axis form with
 * explicit angle and unit axis.
 */

Matrix33d createR(const Vector3d &n, double phi);
double recoverAngleAxis(const Matrix33d &R, Vector3d &n);

/**
 * Conversion between rotation matrix and rotation in angle axis form with
 * length of vector as angle.
 */

inline Matrix33d createR(const Vector3d &n)
{
  // works, because n will be normalized in the createR() function that is called
  return createR(n, norm(n));
}

inline Vector3d recoverAngleAxis(const Matrix33d &R)
{
  Vector3d v;
  double phi=recoverAngleAxis(R, v);
  return v*phi;
}

/**
 * Computes the average rotation as the rotation that with the lowest difference
 * rotation to all given rotations.
 *
 * All rotations are expected in angle axis form.
 */

Vector3d averageAngleAxis(int n, Vector3d v[]);

/**
 * Returns a skew symmetric 3x3 matrix that is constructed by the elements of
 * the given vector. The product the resulting matrix with another vector b is
 * the same as the cross product of vector a and b, i.e. a x b=[a]x * b.
 */

inline Matrix33d createSkewSymmetric(const Vector3d &a)
{
  Matrix33d S;

  S(0, 0)=0;     S(0, 1)=-a[2]; S(0, 2)=a[1];
  S(1, 0)=a[2];  S(1, 1)=0;     S(1, 2)=-a[0];
  S(2, 0)=-a[1]; S(2, 1)=a[0];  S(2, 2)=0;

  return S;
}

/**
 * Applies the Gauss-Jordan algorithm with partial pivoting to the given
 * matrix. Returned is the product of all pivot elements, which is
 * additionally multiplied by -1 for each row swap. Thus, if a square matrix
 * is given, the return value is the determinant.
 */

double transformGaussJordan(Matrixd &a);

/**
 * Determinant of matrix.
 */

template<class T> inline T det(const SMatrix<T, 2, 2> &a)
{
  return a(0, 0)*a(1, 1)-a(0, 1)*a(1, 0);
}

template<class T> inline T det(const SMatrix<T, 3, 3> &a)
{
  return a(0, 0)*a(1, 1)*a(2, 2)+a(1, 0)*a(2, 1)*a(0, 2)+a(2, 0)*a(0, 1)*a(1, 2)-
         a(0, 2)*a(1, 1)*a(2, 0)-a(1, 2)*a(2, 1)*a(0, 0)-a(2, 2)*a(0, 1)*a(1, 0);
}

double det(const Matrixd &a);

/**
 * Returns the inverse of a 3x3 matrix.
 */

Matrix33d inv(const Matrix33d &a);

// --- Singular Value Decomposition ---

/**
   Singular Value Decomposition (SVD), which computes U, W and V such that
   A=U * Matrixd(U.cols(), V.rows(), W) * V^T from the MxN matrix A. The
   singular elements in the vector W are non-negative and sorted in decreasing
   order. If thin == false (i.e. default), then U will be an MxM matrix and V
   will be an NxN matrix. If thin == true, then U will be an M x std::min(M, N)
   matrix.

   The implementation is based on the Golub-Kahan-Reinsch algorithm. It is
   rather fast, compared to the Jacobi method, but in contrast to the Jacobi
   method, the absolute error in the singular elements is constant. The largest
   singular element has almost machine precision. Smaller singular values have
   larger relative errors. Use another SVD implementation, if the error of
   small singular values is critical.
*/

void svd(const Matrixd &a, Matrixd &u, Vectord &w, Matrixd &v,
         bool thin=false);

/**
   Wrapper for using SVD on static matrices.
*/

template<class T, int m, int n> inline void svd(const SMatrix<T, m, n> &a,
    SMatrix<T, m, m> &u, SVector<T, n> &w, SMatrix<T, n, n> &v)
{
  Matrixd U, V;
  Vectord W;

  svd(a, U, W, V, false);
  u=U; v=V;

  int nn=std::min(w.size(), W.size());

  for (int i=0; i<nn; i++)
  {
    w[i]=W[i];
  }

  for (int i=nn; i < w.size(); i++)
  {
    w[i]=0;
  }
}

/**
   Sets all singular values w[i] exactly to 0, for which w[i]/max(w[i])<=t.
   This ensures that the matrix is not ill conditioned. The threshold t should
   be larger than the machine precision, i.e. 1e-16.
*/

inline void svToZero(Vectord &w, double t=1e-15)
{
  // get largest singular value

  double wmax=w[0];

  for (int i=1; i<w.size(); i++)
  {
    wmax=std::max(wmax, w[i]);
  }

  // set all elements below threshold to zero

  for (int i=0; i<w.size(); i++)
  {
    if (w[i]/wmax <= t)
    {
      w[i]=0;
    }
  }
}

/**
   Efficient multiplication of U * Matrixd(U.cols(), V.rows(), W) * V^T.
*/

inline Matrixd mul(const Matrixd &u, const Vectord &w, const Matrixd &v)
{
  Matrixd wv(u.cols(), v.rows());

  int mm=std::min(v.cols(), std::min(u.cols(), w.size()));

  for (int k=0; k<mm; k++)
  {
    for (int i=0; i<v.rows(); i++)
    {
      wv(k, i)=w[k]*v(i, k);
    }
  }

  for (int k=mm; k<u.cols(); k++)
  {
    for (int i=0; i<v.rows(); i++)
    {
      wv(k, i)=0;
    }
  }

  return u*wv;
}

/**
   Efficient multiplication using static matrices.
*/

template<class T, int m, int n, int nn> inline SMatrix<T, m, n>
mul(const SMatrix<T, m, m> &u, const SVector<T, nn> &w,
    const SMatrix<T, n, n> &v)
{
  SMatrix<T, m, n> wv;

  int mm=std::min(v.cols(), std::min(u.cols(), w.size()));

  for (int k=0; k<mm; k++)
  {
    for (int i=0; i<v.rows(); i++)
    {
      wv(k, i)=w[k]*v(i, k);
    }
  }

  for (int k=mm; k<u.cols(); k++)
  {
    for (int i=0; i<v.rows(); i++)
    {
      wv(k, i)=0;
    }
  }

  return u*wv;
}

/**
   Returns the inverse matrix. NOTE: This function throws an
   InvalidArgumentException, if the matrix is not invertable.
*/

inline Matrixd inv(const Matrixd &a)
{
  Matrixd u, v;
  Vectord w;

  assert(a.rows() == a.cols());

  svd(a, u, w, v, true);

  for (int i=0; i<w.size(); i++)
  {
    if (w[i] == 0)
    {
      throw gutil::InvalidArgumentException("Matrix cannot be inverted, because it is singular");
    }

    w[i]=1.0/w[i];
  }

  return mul(v, w, u);
}

/**
   Returns the pseudo inverse matrix, computed by SVD.
*/

inline Matrixd pinv(const Matrixd &a)
{
  Matrixd u, v;
  Vectord w;

  svd(a, u, w, v, true);

  for (int i=0; i<w.size(); i++)
  {
    if (w[i] != 0)
    {
      w[i]=1.0/w[i];
    }
  }

  return mul(v, w, u);
}

/**
   Returns the pseudo inverse of a static matrix.
*/

template<class T, int m, int n> inline SMatrix<T, n, m> pinv(
  const SMatrix<T, m, n> &a)
{
  return pinv(Matrixd(a));
}

}

#endif
