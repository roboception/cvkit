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

#ifndef GMATH_POLYNOMIAL_H
#define GMATH_POLYNOMIAL_H

#include <ios>
#include <algorithm>
#include <vector>
#include <cmath>

namespace gmath
{

/**
 * This template defines a polynomial, which is defined through its
 * coefficients: y=c[0] + c[1]*x + c[2]*x^2 + ... + c[n] * x^n
 */

template<class T> class Polynomial
{
  private:
  
    int len;
    int used;
    T   *c;
    
    void increase(int n)
    {
      if (n > len)
      {
        n+=5;
        T *v=new T[n];
        
        for (int i=0; i<used; i++)
          v[i]=c[i];
        
        delete [] c;
        
        len=n;
        c=v;
      }
    }
    
    void reduce()
    {
        // I am not sure if this is correct, but it seems to work
      
      T v=0;
      
      for (int i=0; i<used; i++)
        v=std::max(v, std::abs(c[i]));
      
      if (v == 0)
        v=1e-15;
      
      while (used > 1 && std::abs(c[used-1])/v < 1e-15)
        used--;
   }
    
  public:
  
    Polynomial()
    {
      len=12;
      used=0;
      c=new T[len];
      c[used++]=0;
    }
    
    Polynomial(T c0, T c1=0, T c2=0, T c3=0, T c4=0)
    {
      len=12;
      used=0;
      c=new T[len];
      c[used++]=c0;
      c[used++]=c1;
      c[used++]=c2;
      c[used++]=c3;
      c[used++]=c4;
      reduce();
    }
    
    Polynomial(const Polynomial<T> &p)
    {
      len=std::max(12, p.getDegree()+1);
      used=p.getDegree()+1;
      c=new T[len];
      
      for (int i=0; i<=p.getDegree(); i++)
        c[i]=p.c[i];
    }
    
    ~Polynomial()
    {
      delete [] c;
    }
    
    void init(T c0=0)
    {
      used=0;
      c[used++]=c0;
    }
    
    int getDegree() const
    {
      return used-1;
    }
    
    T operator()(T x) const
    {
      int i;
      T   ret=0;
      
      for (i=used-1; i>=0; i--)
        ret=x*ret+c[i];
      
      return ret;
    }
    
    T operator[](int i) const
    {
      if (i < used)
        return c[i];
      
      return 0;
    }
    
    void set(int i, T ci)
    {
      increase(i+1);
      for (int k=used; k<i; k++)
        c[k]=0;
      
      c[i]=ci;
      used=std::max(used, i+1);
      
      reduce();
    }
    
    Polynomial<T>& operator=(const Polynomial<T> &p)
    {
      increase(p.getDegree()+1);
      
      for (used=0; used<=p.getDegree(); used++)
        c[used]=p.c[used];
      
      return *this;
    }
    
    Polynomial<T>& operator+=(const Polynomial<T> &p)
    {
      increase(p.getDegree()+1);
      
      for (int i=0; i<used; i++)
        c[i]+=p[i];
      
      for (; used<=p.getDegree(); used++)
        c[used]=p.c[used];
      
      reduce();
      
      return *this;
    }
    
    Polynomial<T> operator+(const Polynomial<T> &p) const
    {
      Polynomial<T> ret;
      
      ret.increase(max(used, p.getDegree()+1));
      ret.used=max(used, p.getDegree()+1);
      
      for (int i=0; i<used; i++)
        ret.c[i]=c[i]+p[i];
      
      for (int i=used; i<=p.getDegree(); i++)
        ret.c[i]=p.c[i];
      
      ret.reduce();
      
      return ret;
    }
    
    Polynomial<T>& operator-=(const Polynomial<T> &p)
    {
      increase(p.getDegree()+1);
      
      for (int i=0; i<used; i++)
        c[i]-=p[i];
      
      for (; used<=p.getDegree(); used++)
        c[used]=-p.c[used];
      
      reduce();
      
      return *this;
    }
    
    Polynomial<T> operator-(const Polynomial<T> &p) const
    {
      Polynomial<T> ret;
      
      ret.increase(max(used, p.getDegree()+1));
      ret.used=max(used, p.getDegree()+1);
      
      for (int i=0; i<used; i++)
        ret.c[i]=c[i]-p[i];
      
      for (int i=used; i<=p.getDegree(); i++)
        ret.c[i]=-p.c[i];
      
      ret.reduce();
      
      return ret;
    }
    
    Polynomial<T> operator-() const
    {
      Polynomial<T> ret;
      
      ret.increase(used);
      ret.used=used;
      
      for (int i=0; i<used; i++)
        ret.c[i]=-c[i];
      
      return ret;
    }
    
    Polynomial<T>& operator*=(T s)
    {
      for (int i=0; i<used; i++)
        c[i]*=s;
      
      return *this;
    }
    
    Polynomial<T> operator*(T s) const
    {
      Polynomial<T> ret;
      
      ret.increase(used);
      ret.used=used;
      
      for (int i=0; i<used; i++)
        ret.c[i]=c[i]*s;
      
      return ret;
    }
    
    Polynomial<T>& operator*=(const Polynomial<T> &p)
    {
      int n=used+p.getDegree();
      T   *cc=new T[n];
      
      for (int i=0; i<n; i++)
        cc[i]=0;
      
      for (int k=0; k<=p.getDegree(); k++)
      {
        if (p.c[k] != 0)
        {
          for (int i=0; i<used; i++)
            cc[i+k]+=c[i]*p[k];
        }
      }
      
      delete [] c;
      len=used=n;
      c=cc;
      
      reduce();
      
      return *this;
    }
    
    Polynomial<T> operator*(const Polynomial<T> &p) const
    {
      Polynomial<T> ret;
      
      ret.increase(used+p.getDegree());
      ret.used=used+p.getDegree();
      
      for (int i=0; i<ret.used; i++)
        ret.c[i]=0;
      
      for (int k=0; k<=p.getDegree(); k++)
      {
        if (p.c[k] != 0)
        {
          for (int i=0; i<used; i++)
            ret.c[i+k]+=c[i]*p[k];
        }
      }
      
      ret.reduce();
      
      return ret;
    }
    
    Polynomial<T>& operator/=(T s)
    {
      for (int i=0; i<used; i++)
        c[i]/=s;
      
      return *this;
    }
    
    Polynomial<T> operator/(T s) const
    {
      Polynomial<T> ret;
      
      ret.increase(used);
      ret.used=used;
      
      for (int i=0; i<used; i++)
        ret.c[i]=c[i]/s;
      
      return ret;
    }
};

template<class T, class S> inline Polynomial<T> operator*(S s, const Polynomial<T> &p)
{
    Polynomial<T> ret;
    
    for (int i=0; i<=p.getDegree(); i++)
      ret.set(i, s*p[i]);
    
    return ret;
}

/**
 * Polynomial division with remainder, i.e. p/d=q+r/d
 */

template<class T> void div(const Polynomial<T> &p, const Polynomial<T> &d,
  Polynomial<T> &q, Polynomial<T> &r)
{
      // initialisation
    
    q.init();
    r=p;
    
      // perform division
    
    for (int k=p.getDegree()-d.getDegree(); k>=0; k--)
    {
      q.set(k, r[d.getDegree()+k]/d[d.getDegree()]);
      
      for (int i=d.getDegree()+k-1; i>=k; i--)
        r.set(i, r[i]-q[k]*d[i-k]);
    }
    
      // set leading coefficients of working array zero
    
    for (int i=d.getDegree(); i<p.getDegree()+1; i++)
      r.set(i, 0);
}

/**
 * Computes the derivation of p.
 */

template<class T> inline Polynomial<T> d(const Polynomial<T> &p)
{
    Polynomial<T> ret;
    
    for (int i=1; i<=p.getDegree(); i++)
      ret.set(i-1, i*p[i]);
    
    return ret;
}

template<class T, class Ch, class Tr>
std::basic_ostream<Ch, Tr>& operator<<(std::basic_ostream<Ch, Tr> &out, const Polynomial<T> &p)
{
    for (int i=p.getDegree(); i>=0; i--)
    {
      if (p[i] != 0 || p.getDegree() == 0)
      {
        if (i < p.getDegree())
        {
          if (p[i] < 0)
            out << " - ";
          else
            out << " + ";
        }
        else
        {
          if (p[i] < 0)
            out << "-";
        }
        
        if (std::abs(p[i]) != 1 || i == 0)
        {
          out << std::abs(p[i]);
          
          if (i > 0)
            out << "*";
        }
        
        if (i >= 2)
          out << "x^" << i;
        else if (i == 1)
          out << "x";
      }
    }
    
    return out;
}

typedef Polynomial<double> Polynomiald;

std::vector<double> realRoots(const Polynomiald &p);

}

#endif
