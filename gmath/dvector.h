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

#ifndef GMATH_DVECTOR_H
#define GMATH_DVECTOR_H

#include "svector.h"

#include <gutil/exception.h>

#include <ios>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <vector>

#include <assert.h>

namespace gmath
{


/**
 * This template defines a dynamic vector with all basic mathematical functions
 * for vector arithmetic. The size of the vector is defined at run-time, which
 * makes this more flexible than static vectors (i.e. SVector), but also slower.
 *
 * Higher level operations which are more complex, are only defined for dynamic
 * vectors. Construction of dynamic vectors from static vectors and casting into
 * static vectors is defined for making all complex operations available to
 * static vectors too. Most conversions will be done implicitely by the
 * compiler.
 */

template<class T> class DVector
{
  private:
    int n;
    T   *v;
    
    void checkDimension(int m) const
    {
      assert(n == m);
    }
    
  public:
    explicit DVector(int len=0)
    {
      n=0;
      v=0;
      
      init(len);
    }
    
    DVector(T v0, T v1)
    {
      n=2;
      v=new T[2];
      
      v[0]=v0;
      v[1]=v1;
    }
    
    DVector(T v0, T v1, T v2)
    {
      n=3;
      v=new T[3];
      
      v[0]=v0;
      v[1]=v1;
      v[2]=v2;
    }
    
    DVector(T v0, T v1, T v2, T v3)
    {
      n=4;
      v=new T[4];
      
      v[0]=v0;
      v[1]=v1;
      v[2]=v2;
      v[3]=v3;
    }
    
    DVector(const DVector<T> &a)
    {
      n=a.size();
      
      v=0;
      if (n > 0)
      {
        v=new T[n];
        
        for (int i=0; i<n; i++)
          v[i]=a[i];
      }
    }
    
    template<class S, int m> DVector(const SVector<S, m> &a)
    {
      n=m;
      v=new T[n];
      
      for (int i=0; i<m; i++)
        v[i]=a[i];
    }
    
    ~DVector()
    {
      delete [] v;
    }
    
    void init(int len)
    {
      if (v != 0)
        delete [] v;
      
      n=len;
      
      v=0;
      if (n > 0)
      {
        v=new T[n];
        
        for (int i=0; i<n; i++)
          v[i]=0;
      }
    }
    
    int size() const
    {
      return n;
    }
    
    T& operator[](int i)
    {
      return v[i];
    }
    
    T operator[](int i) const
    {
      return v[i];
    }
    
    DVector<T>& operator=(const DVector<T> &a)
    {
      if (v != 0)
        delete [] v;
      
      n=a.size();
      
      v=0;
      if (n > 0)
      {
        v=new T[n];
        
        for (int i=0; i<n; i++)
          v[i]=a[i];
      }
      
      return *this;
    }
    
    template<class S, int m> operator SVector<S, m>() const
    {
      SVector<S, m> ret;
      
      assert(m == n);
      
      for (int i=0; i<m; i++)
        ret[i]=static_cast<S>(v[i]);
      
      return ret;
    }
    
    DVector<T>& operator+=(const DVector<T> &a)
    {
      checkDimension(a.n);
      
      for (int i=0; i<n; i++)
        v[i]+=a.v[i];
      
      return *this;
    }
    
    DVector<T> operator+(const DVector<T> &a) const
    {
      DVector<T> ret(n);
      
      checkDimension(a.n);
      
      for (int i=0; i<n; i++)
        ret.v[i]=v[i]+a.v[i];
      
      return ret;
    }
    
    DVector<T>& operator-=(const DVector<T> &a)
    {
      checkDimension(a.n);
      
      for (int i=0; i<n; i++)
        v[i]-=a.v[i];
      
      return *this;
    }
    
    DVector<T> operator-() const
    {
      DVector<T> ret(n);
      
      for (int i=0; i<n; i++)
        ret.v[i]=-v[i];
      
      return ret;
    }
    
    DVector<T> operator-(const DVector<T> &a) const
    {
      DVector<T> ret(n);
      
      checkDimension(a.n);
      
      for (int i=0; i<n; i++)
        ret.v[i]=v[i]-a.v[i];
      
      return ret;
    }
    
    DVector<T>& operator*=(T s)
    {
      for (int i=0; i<n; i++)
        v[i]*=s;
      
      return *this;
    }
    
    DVector<T> operator*(T s) const
    {
      DVector<T> ret(n);
      
      for (int i=0; i<n; i++)
        ret.v[i]=s*v[i];
      
      return ret;
    }
    
    T operator*(const DVector<T> &a) const
    {
      T ret=0;
      
      checkDimension(a.n);
      
      for (int i=0; i<n; i++)
        ret+=v[i]*a.v[i];
      
      return ret;
    }
    
    DVector<T>& operator/=(T s)
    {
      for (int i=0; i<n; i++)
        v[i]/=s;
      
      return *this;
    }
    
    DVector<T> operator/(T s) const
    {
      DVector<T> ret(n);
      
      for (int i=0; i<n; i++)
        ret.v[i]=v[i]/s;
      
      return ret;
    }
    
    bool operator==(const DVector<T> &a) const
    {
      if (n != a.n)
        return false;
      
      for (int i=0; i<n; i++)
        if (v[i] != a.v[i])
          return false;
      
      return true;
    }
    
    bool operator!=(const DVector<T> &a) const
    {
      if (n != a.n)
        return true;
      
      for (int i=0; i<n; i++)
        if (v[i] != a.v[i])
          return true;
      
      return false;
    }
};

template<class S, class T> inline DVector<T> operator*(S s, const DVector<T> &a)
{
    DVector<T> ret(a.size());
    
    for (int i=0; i<a.size(); i++)
      ret[i]=s*a[i];
    
    return ret;
}

template<class T> inline DVector<T> cross(const DVector<T> &a, const DVector<T> &b)
{
    DVector<T> ret(a.size());
    
    assert(a.size() == 3);
    assert(b.size() == 3);
    
    ret[0]=a[1]*b[2]-a[2]*b[1];
    ret[1]=a[2]*b[0]-a[0]*b[2];
    ret[2]=a[0]*b[1]-a[1]*b[0];
    
    return ret;
}

template<class T> inline T norm(const DVector<T> &a)
{
    T ret=0;
    
    for (int i=0; i<a.size(); i++)
      ret+=a[i]*a[i];
    
    return sqrt(ret);
}

template<class T, class Ch, class Tr>
std::basic_ostream<Ch, Tr>& operator<<(std::basic_ostream<Ch, Tr> &out, const DVector<T> &a)
{
    out << "[";
    
    for (int i=0; i<a.size(); i++)
    {
      if (i > 0)
        out << " ";
      
      out << a[i];
    }
    
    out << "]";
    
    return out;
}

template<class T, class Ch, class Tr>
std::basic_istream<Ch, Tr>& operator>>(std::basic_istream<Ch, Tr> &in, DVector<T> &a)
{
    T              v;
    std::vector<T> list;
    char           c;
    
    in >> c;
    
    if (c == '[')
    {
      while (c != ']' && in)
      {
        in >> c;
        
        if (c != ']')
        {
          in.putback(c);
          
          in >> v;
          list.push_back(v);
        }
      }
      
      if (in)
      {
        a.init(list.size());
        
        for (int i=0; i<a.size(); i++)
          a[i]=list[i];
      }
    }
    else
      in.setstate(std::ios_base::failbit);
    
    return in;
}

typedef DVector<double> Vectord;

}

#endif
