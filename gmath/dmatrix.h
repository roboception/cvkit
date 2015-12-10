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

#ifndef GMATH_DMATRIX_H
#define GMATH_DMATRIX_H

#include "dvector.h"
#include "smatrix.h"

#include <iostream>

#include <gutil/exception.h>

#include <ios>
#include <stdexcept>
#include <sstream>
#include <vector>

#include <assert.h>

namespace gmath
{

/**
 * This template defines a dynamic matrix with all basic mathematical functions
 * for matrix arithmetic. The size of the matrix is defined at run-time, which
 * makes this more flexible than static matrices (i.e. SMatrix), but also
 * slower.
 *
 * Higher level operations which are more complex, are only defined for dynamic
 * matrices. Construction of dynamic matrices from static matrices and casting
 * into static matrices is defined for making all complex operations available
 * to static matrices too. Most conversions will be done implicitely by the
 * compiler.
 */

template<class T> class DMatrix
{
  private:
    int nrows, ncols, n;
    T   *v;
    
    void checkDimension(int r, int c) const
    {
      assert(nrows == r);
      assert(ncols == c);
    }
    
  public:
    
    /**
     * Initialisation with identity matrix
     */
    
    DMatrix(int r=0, int c=0)
    {
      nrows=0;
      ncols=0;
      n=0;
      v=0;
      
      init(r, c);
    }
    
    /**
     * Initialisation with elements from a vector. The elements are put onto
     * the main diagonal of the matrix. If the vector is too large, then
     * additional values are ignored. If it is too short, then missing elements
     * are replaced by 0.
     */
    
    DMatrix(int r, int c, const DVector<T> &a)
    {
      nrows=r;
      ncols=c;
      n=nrows*ncols;
      v=new T[n];
      
      for (int i=0; i<n; i++)
        v[i]=0;
      
      int m=min(ncols, min(nrows, a.size()));
      for (int i=0; i<m; i++)
        v[i*ncols+i]=a[i];
    }
    
    DMatrix(const DMatrix<T> &a)
    {
      nrows=a.rows();
      ncols=a.cols();
      n=nrows*ncols;
      
      v=0;
      if (n > 0)
      {
        v=new T[n];
        
        for (int i=0; i<n; i++)
          v[i]=a.v[i];
      }
    }
    
    template<class S, int r, int c> DMatrix(const SMatrix<S, r, c> &a)
    {
      nrows=r;
      ncols=c;
      n=r*c;
      v=new T[n];
      
      for (int k=0; k<r; k++)
        for (int i=0; i<c; i++)
          v[k*c+i]=a(k, i);
    }
    
    ~DMatrix()
    {
      delete [] v;
    }
    
    /**
     * Initialisation with identity matrix.
     */
    
    void init(int rows=0, int cols=0)
    {
      if (v != 0)
        delete [] v;
      
      nrows=rows;
      ncols=cols;
      n=nrows*ncols;
      
      v=0;
      if (n > 0)
      {
        v=new T[n];
        
        for (int i=0; i<n; i++)
          v[i]=0;
        
        for (int i=0; i<n; i+=cols+1)
          v[i]=1;
      }
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
      return v[k*ncols+i];
    }
    
    T operator()(int k, int i) const
    {
      return v[k*ncols+i];
    }
    
    DVector<T> getRow(int row) const
    {
      DVector<T> ret(ncols);
      
      for (int i=0; i<ncols; i++)
        ret[i]=v[row*ncols+i];
      
      return ret;
    }
    
    DVector<T> getColumn(int col) const
    {
      DVector<T> ret(nrows);
      
      for (int k=0; k<nrows; k++)
        ret[k]=v[k*ncols+col];
      
      return ret;
    }
    
    void setRow(int row, const DVector<T> &a)
    {
      checkDimension(nrows, a.size());
      
      for (int i=0; i<ncols; i++)
        v[row*ncols+i]=a[i];
    }
    
    void setColumn(int col, const DVector<T> &a)
    {
      checkDimension(a.size(), ncols);
      
      for (int k=0; k<nrows; k++)
        v[k*ncols+col]=a[k];
    }
    
    DMatrix<T>& operator=(const DMatrix<T> &a)
    {
      if (v != 0)
        delete [] v;
      
      nrows=a.nrows;
      ncols=a.ncols;
      n=nrows*ncols;
      
      v=0;
      if (n > 0)
      {
        v=new T[n];
        
        for (int i=0; i<n; i++)
          v[i]=a.v[i];
      }
      
      return *this;
    }
    
    DMatrix<T>& operator=(T s)
    {
      for (int i=0; i<n; i++)
        v[i]=s;
      
      return *this;
    }
    
    template<class S, int r, int c> operator SMatrix<S, r, c>() const
    {
      SMatrix<S, r, c> ret;
      
      assert(nrows == r);
      assert(ncols == c);
      
      for (int k=0; k<r; k++)
        for (int i=0; i<c; i++)
          ret(k, i)=static_cast<S>(v[k*c+i]);
      
      return ret;
    }
    
    DMatrix<T>& operator+=(const DMatrix<T> &a)
    {
      checkDimension(a.nrows, a.ncols);
      
      for (int i=0; i<n; i++)
        v[i]+=a.v[i];
      
      return *this;
    }
    
    DMatrix<T> operator+(const DMatrix<T> &a) const
    {
      DMatrix<T> ret(nrows, ncols);
      
      checkDimension(a.nrows, a.ncols);
      
      for (int i=0; i<n; i++)
        ret.v[i]=v[i]+a.v[i];
      
      return ret;
    }
    
    DMatrix<T>& operator-=(const DMatrix<T> &a)
    {
      checkDimension(a.nrows, a.ncols);
      
      for (int i=0; i<n; i++)
        v[i]-=a.v[i];
      
      return *this;
    }
    
    DMatrix<T> operator-(const DMatrix<T> &a) const
    {
      DMatrix<T> ret(nrows, ncols);
      
      checkDimension(a.nrows, a.ncols);
      
      for (int i=0; i<n; i++)
        ret.v[i]=v[i]-a.v[i];
      
      return ret;
    }
    
    DMatrix<T>& operator*=(T s)
    {
      for (int i=0; i<n; i++)
        v[i]*=s;
      
      return *this;
    }
    
    DMatrix<T> operator*(T s) const
    {
      DMatrix<T> ret(nrows, ncols);
      
      for (int i=0; i<n; i++)
        ret.v[i]=s*v[i];
      
      return ret;
    }
    
    DVector<T> operator*(const DVector<T> &a) const
    {
      DVector<T> ret(nrows);
      
      checkDimension(nrows, a.size());
      
      for (int k=0; k<nrows; k++)
        for (int i=0; i<ncols; i++)
          ret[k]+=v[k*ncols+i]*a[i];
      
      return ret;
    }
    
    DMatrix<T>& operator/=(T s)
    {
      for (int i=0; i<n; i++)
        v[i]/=s;
      
      return *this;
    }
    
    DMatrix<T> operator/(T s) const
    {
      DMatrix<T> ret(nrows, ncols);
      
      for (int i=0; i<n; i++)
        ret.v[i]=v[i]/s;
      
      return ret;
    }
    
    bool operator==(const DMatrix<T> &a) const
    {
      for (int i=0; i<n; i++)
        if (v[i] != a.v[i])
          return false;
      
      return true;
    }
    
    bool operator!=(const DMatrix<T> &a) const
    {
      for (int i=0; i<n; i++)
        if (v[i] != a.v[i])
          return true;
      
      return false;
    }
};

template<class T> DMatrix<T> inline operator*(const DMatrix<T> &a, const DMatrix<T> &b)
{
    DMatrix<T> ret(a.rows(), b.cols());
    
    assert(a.cols() == b.rows());
    
    for (int k=0; k<a.rows(); k++)
    {
      for (int i=0; i<b.cols(); i++)
      {
        ret(k, i)=0;
        for (int j=0; j<a.cols(); j++)
          ret(k, i)+=a(k, j)*b(j, i);
      }
    }
    
    return ret;
}

/**
 * Multiplication of a vector (which is implicitely transposed) with a matrix.
 * The resulting vector is implicitely transposed again, i.e. c'=a'*b
 */

template<class T> inline DVector<T> operator*(const DVector<T> &a,
  const DMatrix<T> &b)
{
    DVector<T> ret(b.cols());
    
    assert(a.size() == b.rows());
    
    for (int i=0; i<b.cols(); i++)
      for (int k=0; k<b.rows(); k++)
        ret[i]+=a[k]*b(k, i);
    
    return ret;
}

template<class S, class T> DMatrix<T> inline operator*(S s, const DMatrix<T> &a)
{
    DMatrix<T> ret(a.rows(), a.cols());
    
    for (int k=0; k<ret.rows(); k++)
      for (int i=0; i<ret.cols(); i++)
        ret(k, i)=s*a(k, i);
    
    return ret;
}

template<class T> inline DMatrix<T> transpose(const DMatrix<T> &a)
{
    DMatrix<T> ret(a.cols(), a.rows());
    
    for (int k=0; k<ret.rows(); k++)
      for (int i=0; i<ret.cols(); i++)
        ret(k, i)=a(i, k);
    
    return ret;
}

template<class T, class Ch, class Tr>
std::basic_ostream<Ch, Tr>& operator<<(std::basic_ostream<Ch, Tr> &out, const DMatrix<T> &a)
{
    out << "[";
    
    for (int k=0; k<a.rows(); k++)
    {
      for (int i=0; i<a.cols(); i++)
      {
        if (i > 0)
          out << " ";
        
        out << a(k, i);
      }
      
      if (k+1 < a.rows())
        out << "; ";
    }
    
    out << "]";
    
    return out;
}

template<class T, class Ch, class Tr>
std::basic_istream<Ch, Tr>& operator>>(std::basic_istream<Ch, Tr> &in, DMatrix<T> &a)
{
    T    v;
    std::vector<std::vector<T> > elem;
    char c;
    
    in >> c;
    
    if (c == '[')
    {
      while (c != ']' && in)
      {
        in >> c;
        if (c != ']')
        {
          if (c != ';')
            in.putback(c);
          
          elem.push_back(std::vector<T>());
          
          while (c != ';' && c != ']' && in)
          {
            in >> c;
            if (c != ';' && c != ']')
            {
              in.putback(c);
              
              in >> v;
              elem.back().push_back(v);
            }
          }
          
          if (in && elem.front().size() != elem.back().size())
            in.setstate(std::ios_base::failbit);
        }
      }
      
      if (in)
      {
        int rows=elem.size();
        int cols=0;
        
        if (rows > 0)
          cols=elem.front().size();
        
        a.init(rows, cols);
        
        for (int k=0; k<rows; k++)
          for (int i=0; i<cols; i++)
            a(k, i)=elem[k][i];
      }
    }
    else
      in.setstate(std::ios_base::failbit);
    
    return in;
}

typedef DMatrix<double> Matrixd;

}

#endif
