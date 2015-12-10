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

#ifndef GMATH_SVECTOR_H
#define GMATH_SVECTOR_H

#include <gutil/exception.h>

#include <ios>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <assert.h>

namespace gmath
{

/**
 * This template defines a static vector with all basic mathematical functions
 * for vector arithmetic. The size of the vector is predefined at compile time
 * by a template parameter. This allows the compiler to generate highly
 * efficient code with straight forward optimizations. All checks for vector
 * sizes are done at compile time, there is no allocation of dynamic memory,
 * unrolling of static loops is simple and inlining eliminates function calls.
 *
 * The code generation overhead of instantiating a vector template not only for
 * each data type, but also for each vector size is justified by the fact that
 * it is sufficient for most computer vision algorithms to use the type double
 * with three kinds of vector sizes, i.e. 2 for describing a point in an image
 * plane, 3 for describing a point in Euclidean space and 4 for a point in
 * projective space. See the corresponding typedefs Vector2d, Vector3d and
 * Vector4d.
 *
 * If higher dimensional vectors or definition of its size at run-time is
 * needed, a DVector (i.e. dynamic vector) should be used instead. Conversions
 * are available in the DVector template.
 */

template<class T, int n> class SVector
{
  private:
    T v[n];
  
  public:
    SVector()
    {
      for (int i=0; i<n; i++)
        v[i]=0;
    }
    
    SVector(T v0, T v1)
    {
      assert(n == 2);
      
      v[0]=v0;
      v[1]=v1;
    }
    
    SVector(T v0, T v1, T v2)
    {
      assert(n == 3);
      
      v[0]=v0;
      v[1]=v1;
      v[2]=v2;
    }
    
    SVector(T v0, T v1, T v2, T v3)
    {
      assert(n == 4);
      
      v[0]=v0;
      v[1]=v1;
      v[2]=v2;
      v[3]=v3;
    }
    
    SVector(T v0, T v1, T v2, T v3, T v4, T v5)
    {
      assert(n == 6);
      
      v[0]=v0;
      v[1]=v1;
      v[2]=v2;
      v[3]=v3;
      v[4]=v4;
      v[5]=v5;
    }
    
    template<class S> SVector(const SVector<S, n> &a)
    {
      for (int i=0; i<n; i++)
        v[i]=a[i];
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
    
    SVector<T, n>& operator=(T s)
    {
      for (int i=0; i<n; i++)
        v[i]=s;
      
      return *this;
    }
    
    SVector<T, n>& operator=(const SVector<T, n> &a)
    {
      for (int i=0; i<n; i++)
        v[i]=a.v[i];
      
      return *this;
    }
    
    SVector<T, n>& operator+=(const SVector<T, n> &a)
    {
      for (int i=0; i<n; i++)
        v[i]+=a.v[i];
      
      return *this;
    }
    
    SVector<T, n> operator+(const SVector<T, n> &a) const
    {
      SVector<T, n> ret;
      
      for (int i=0; i<n; i++)
        ret.v[i]=v[i]+a.v[i];
      
      return ret;
    }
    
    SVector<T, n>& operator-=(const SVector<T, n> &a)
    {
      for (int i=0; i<n; i++)
        v[i]-=a.v[i];
      
      return *this;
    }
    
    SVector<T, n> operator-() const
    {
      SVector<T, n> ret;
      
      for (int i=0; i<n; i++)
        ret.v[i]=-v[i];
      
      return ret;
    }
    
    SVector<T, n> operator-(const SVector<T ,n> &a) const
    {
      SVector<T, n> ret;
      
      for (int i=0; i<n; i++)
        ret.v[i]=v[i]-a.v[i];
      
      return ret;
    }
    
    SVector<T, n>& operator*=(T s)
    {
      for (int i=0; i<n; i++)
        v[i]*=s;
      
      return *this;
    }
    
    SVector<T, n> operator*(T s) const
    {
      SVector<T, n> ret;
      
      for (int i=0; i<n; i++)
        ret.v[i]=s*v[i];
      
      return ret;
    }
    
    T operator*(const SVector<T, n> &a) const
    {
      T ret=0;
      
      for (int i=0; i<n; i++)
        ret+=v[i]*a.v[i];
      
      return ret;
    }
    
    SVector<T, n>& operator/=(T s)
    {
      for (int i=0; i<n; i++)
        v[i]/=s;
      
      return *this;
    }
    
    SVector<T, n> operator/(T s) const
    {
      SVector<T, n> ret;
      
      for (int i=0; i<n; i++)
        ret.v[i]=v[i]/s;
      
      return ret;
    }
    
    bool operator==(const SVector<T, n> &a) const
    {
      for (int i=0; i<n; i++)
        if (v[i] != a.v[i])
          return false;
      
      return true;
    }
    
    bool operator!=(const SVector<T, n> &a) const
    {
      for (int i=0; i<n; i++)
        if (v[i] != a.v[i])
          return true;
      
      return false;
    }
};

template<class S, class T, int n> inline SVector<T, n> operator*(S s, const SVector<T, n> &a)
{
    SVector<T, n> ret;
    
    for (int i=0; i<n; i++)
      ret[i]=s*a[i];
    
    return ret;
}

template<class T> inline SVector<T, 3> cross (const SVector<T, 3> &a, const SVector<T, 3> &b)
{
    SVector<T, 3> ret;
    
    ret[0]=a[1]*b[2]-a[2]*b[1];
    ret[1]=a[2]*b[0]-a[0]*b[2];
    ret[2]=a[0]*b[1]-a[1]*b[0];
    
    return ret;
}

template<class T, int n> inline T norm(const SVector<T, n> &a)
{
    T ret=0;
    
    for (int i=0; i<n; i++)
      ret+=a[i]*a[i];
    
    return sqrt(ret);
}

template<class T, int n, class Ch, class Tr>
std::basic_ostream<Ch, Tr>& operator<<(std::basic_ostream<Ch, Tr> &out, const SVector<T, n> &a)
{
    out << "[" << a[0];
    
    for (int i=1; i<n; i++)
      out << " " << a[i];
    
    out << "]";
    
    return out;
}

template<class T, int n, class Ch, class Tr>
std::basic_istream<Ch, Tr>& operator>>(std::basic_istream<Ch, Tr> &in, SVector<T, n> &a)
{
    char c;
    
    in >> c;
    
    if (c == '[')
    {
      for (int i=0; i<n && in; i++)
        in >> a[i];
      
      in >> c;
      
      if (c != ']')
        in.setstate(std::ios_base::failbit);
    }
    else
      in.setstate(std::ios_base::failbit);
    
    return in;
}

typedef SVector<double, 2> Vector2d;
typedef SVector<double, 3> Vector3d;
typedef SVector<double, 4> Vector4d;
typedef SVector<double, 6> Vector6d;

}

#endif
