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

#ifndef GMATH_SMATRIX_H
#define GMATH_SMATRIX_H

#include "svector.h"

#include <ios>

namespace gmath
{

/**
 * This template defines a static matrix with all basic mathematical functions
 * for matrix arithmetic. The size of the matrix is predefined at compile time
 * by template parameters. This allows the compiler to generate highly
 * efficient code with straight forward optimizations. All checks for matrix and
 * vector sizes are done at compile time, there is no allocation of dynamic
 * memory, unrolling of static loops is simple and inlining eliminates function
 * calls.
 *
 * The code generation overhead of instantiating a matrix template not only for
 * each data type, but also for each size is justified by the fact that it is
 * sufficient for most computer vision algorithms to use the data type double
 * with matrix sizes of 3x3 for rotations in Euclidean space or 3x4 for
 * projection matrices. See the corresponding typedefs Matrix33d and Matrix34d.
 *
 * If higher dimensional matrices, definition of its size at run-time or more
 * complex operations are needed, a DMatrix (i.e. dynamic matrix) should be used
 * instead. Conversions are available in the DMatrix template.
 */

template<class T, int nrows, int ncols> class SMatrix
{
  private:
    T v[nrows][ncols];
  
  public:
  
    /**
     * Initialisation with identity matrix.
     */
    
    SMatrix()
    {
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          v[k][i]=0;
      
      int n=nrows;
      if (ncols < nrows)
        n=ncols;
      
      for (int i=0; i<n; i++)
        v[i][i]=1;
    }
    
    template<class S> SMatrix(const SMatrix<S, nrows, ncols> &a)
    {
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          v[k][i]=a(k, i);
    }
    
    int rows() const
    {
      return nrows;
    }
    
    int cols() const
    {
      return ncols;
    }
    
    T& operator()(int k, int i)
    {
      return v[k][i];
    }
    
    T operator()(int k, int i) const
    {
      return v[k][i];
    }
    
    SVector<T, ncols> getRow(int row) const
    {
      SVector<T, ncols> ret;
      
      for (int i=0; i<ncols; i++)
        ret[i]=v[row][i];
      
      return ret;
    }
    
    SVector<T, nrows> getColumn(int col) const
    {
      SVector<T, nrows> ret;
      
      for (int k=0; k<nrows; k++)
        ret[k]=v[k][col];
      
      return ret;
    }
    
    void setRow(int row, const SVector<T, ncols> &a)
    {
      for (int i=0; i<ncols; i++)
        v[row][i]=a[i];
    }
    
    void setColumn(int col, const SVector<T, nrows> &a)
    {
      for (int k=0; k<nrows; k++)
        v[k][col]=a[k];
    }
    
    SMatrix<T, nrows, ncols>& operator=(const SMatrix<T, nrows, ncols> &a)
    {
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          v[k][i]=a.v[k][i];
      
      return *this;
    }
    
    SMatrix<T, nrows, ncols>& operator=(T s)
    {
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          v[k][i]=s;
      
      return *this;
    }
    
    SMatrix<T, nrows, ncols>& operator+=(const SMatrix<T, nrows, ncols> &a)
    {
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          v[k][i]+=a.v[k][i];
      
      return *this;
    }
    
    SMatrix<T, nrows, ncols> operator+(const SMatrix<T, nrows, ncols> &a) const
    {
      SMatrix<T, nrows, ncols> ret;
      
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          ret.v[k][i]=v[k][i]+a.v[k][i];
      
      return ret;
    }
    
    SMatrix<T, nrows, ncols>& operator-=(const SMatrix<T, nrows, ncols> &a)
    {
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          v[k][i]-=a.v[k][i];
      
      return *this;
    }
    
    SMatrix<T, nrows, ncols> operator-(const SMatrix<T, nrows, ncols> &a) const
    {
      SMatrix<T, nrows, ncols> ret;
      
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          ret.v[k][i]=v[k][i]-a.v[k][i];
      
      return ret;
    }
    
    SMatrix<T, nrows, ncols>& operator*=(T s)
    {
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          v[k][i]*=s;
      
      return *this;
    }
    
    SMatrix<T, nrows, ncols> operator*(T s) const
    {
      SMatrix<T, nrows, ncols> ret;
      
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          ret.v[k][i]=s*v[k][i];
      
      return ret;
    }
    
    SVector<T, nrows> operator*(const SVector<T, ncols> &a) const
    {
      SVector<T, nrows> ret;
      
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          ret[k]+=v[k][i]*a[i];
      
      return ret;
    }
    
    SMatrix<T, nrows, ncols>& operator/=(T s)
    {
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          v[k][i]/=s;
      
      return *this;
    }
    
    SMatrix<T, nrows, ncols> operator/(T s) const
    {
      SMatrix<T, nrows, ncols> ret;
      
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          ret.v[k][i]=v[k][i]/s;
      
      return ret;
    }
    
    bool operator==(const SMatrix<T, nrows, ncols> &a) const
    {
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          if (v[k][i] != a.v[k][i])
            return false;
      
      return true;
    }
    
    bool operator!=(const SMatrix<T, nrows, ncols> &a) const
    {
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          if (v[k][i] != a.v[k][i])
            return true;
      
      return false;
    }
};

template<class T, int n, int m, int u> inline SMatrix<T, n, m> operator*(const SMatrix<T, n, u> &a,
  const SMatrix<T, u, m> &b)
{
    SMatrix<T, n, m> ret;
    
    for (int k=0; k<n; k++)
    {
      for (int i=0; i<m; i++)
      {
        ret(k, i)=0;
        for (int j=0; j<u; j++)
          ret(k, i)+=a(k, j)*b(j, i);
      }
    }
    
    return ret;
}

/**
 * Multiplication of a vector (which is implicitely transposed) with a matrix.
 * The resulting vector is implicitely transposed again, i.e. c'=a'*b
 */

template<class T, int nrows, int ncols> inline SVector<T, ncols> operator*(const SVector<T, nrows> &a,
  const SMatrix<T, nrows, ncols> &b)
{
    SVector<T, ncols> ret;
    
    for (int i=0; i<ncols; i++)
      for (int k=0; k<nrows; k++)
        ret[i]+=a[k]*b(k, i);
    
    return ret;
}

template<class S, class T, int nrows, int ncols> inline SMatrix<T, nrows, ncols> operator*(S s,
  const SMatrix<T, nrows, ncols> &a)
{
    SMatrix<T, nrows, ncols> ret;
    
    for (int k=0; k<nrows; k++)
      for (int i=0; i<ncols; i++)
        ret(k, i)=s*a(k, i);
    
    return ret;
}

template<class T, int nrows, int ncols> inline SMatrix<T, nrows, ncols> transpose(const SMatrix<T, ncols, nrows> &a)
{
    SMatrix<T, nrows, ncols> ret;
    
    for (int k=0; k<nrows; k++)
      for (int i=0; i<ncols; i++)
        ret(k, i)=a(i, k);
    
    return ret;
}

template<class T, int nrows, int ncols, class Ch, class Tr>
std::basic_ostream<Ch, Tr>& operator<<(std::basic_ostream<Ch, Tr> &out, const SMatrix<T, nrows, ncols> &a)
{
    out << "[";
    
    for (int k=0; k<nrows; k++)
    {
      out << a(k, 0);
      
      for (int i=1; i<ncols; i++)
        out << " " << a(k, i);
      
      if (k+1 < nrows)
        out << "; ";
    }
    
    out << "]";
    
    return out;
}

template<class T, int nrows, int ncols, class Ch, class Tr>
std::basic_istream<Ch, Tr>& operator>>(std::basic_istream<Ch, Tr> &in, SMatrix<T, nrows, ncols> &a)
{
    char c;
    
    in >> c;
    
    if (c == '[')
    {
      for (int k=0; k<nrows && in; k++)
      {
        for (int i=0; i<ncols && in; i++)
          in >> a(k, i);
        
        in >> c;
        
        if (k+1 < nrows && c != ';')
          in.setstate(std::ios_base::failbit);
      }
      
      if (c != ']')
        in.setstate(std::ios_base::failbit);
    }
    else
      in.setstate(std::ios_base::failbit);
    
    return in;
}

typedef SMatrix<double, 3, 3> Matrix33d;
typedef SMatrix<double, 3, 4> Matrix34d;
typedef SMatrix<double, 4, 3> Matrix43d;
typedef SMatrix<double, 4, 4> Matrix44d;

}

#endif
