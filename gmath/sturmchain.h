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

#ifndef GMATH_STURMCHAIN_H
#define GMATH_STURMCHAIN_H

#include "polynomial.h"

#include <gutil/exception.h>
#include <assert.h>

namespace gmath
{

/**
 * This template can create and evaluate a Sturm chain, which is defined by:
 *
 * f0(x) = p(x)     (i.e. the original polynomial)
 * f1(x) = dp(x)/dx (i.e. the first derivation of the polynomial)
 * f2(x) = -r0(x)   (i.e. r0(x) is the remainder of polynomial division of
 *                   f0(x)/f1(x))
 * ...
 * fn-1    = c      (i.e. constant but not 0)
 *
 * The sturm chain is used to determine the number of real roots of a polynomial
 * in a given interval.
 */

template<class T> class SturmChain
{
  private:
    int n;
    Polynomial<T> *f;
    
  public:
    
    SturmChain(const Polynomial<T> &p)
    {
      Polynomial<T> q, r;
      
      assert(p.getDegree() > 0);
      
      n=0;
      f=new Polynomial<T> [p.getDegree()+1];
      
      f[0]=p;
      
      bool compute=true;
      while (compute && f[0].getDegree() > 0)
      {
        compute=false;
        
        f[1]=d(f[0]);
        
        n=2;
        while (n < f[0].getDegree()+1 && f[n-1].getDegree() > 0)
        {
          div(f[n-2], f[n-1], q, r);
          
            // check for multiple roots
          
          if (r.getDegree() == 0 && r[0] == 0)
          {
              // f[n-1] is the common divisor of f[0], divide it out to get
              // rid of multiple roots and restart computation of Sturm chain
            
            div(f[0], f[n-1], q, r);
            f[0]=q;
            compute=true;
            break;
          }
          
          f[n]=-r;
          n++;
        }
      }
    }
    
    ~SturmChain()
    {
      delete [] f;
    }
    
    int size() const
    {
      return n;
    }
    
    const Polynomial<T>& operator[] (int i) const
    {
      return f[i];
    }
    
    /**
     * Number of sign changes of Sturm chain at x.
     */
    
    int countSignChanges(T x) const
    {
      T   f1, f2;
      int ret=0;
      
      f1=0;
      for (int k=0; k<n; k++)
      {
        f2=f[k](x);
        
        if (f1 != 0)
        {
          if (f2 != 0)
          {
            if (f1*f2 < 0)
              ret++;
            
            f1=f2;
          }
        }
        else
          f1=f2;
      }
      
      return ret;
    }
    
    /**
     * Number of sign changes at positive infinity
     */
    
    int countSignChangesPosInf() const
    {
      double f1, f2;
      int    ret=0;
      
        // the sign of the highest coefficient has the same sign as the
        // limes at positive infinity
      
      f1=f[0][f[0].getDegree()];
      for (int k=1; k<n; k++)
      {
        f2=f[k][f[k].getDegree()];
        
        if ((f1 < 0 && f2 >= 0) || (f1 >= 0 && f2 < 0))
          ret++;
        
        f1=f2;
      }
      
      return ret;
    }
    
    /**
     * Number of sign changes at negative infinity
     */
    
    int countSignChangesNegInf() const
    {
      double f1, f2;
      int    ret=0;
      
        // the sign of the highest coefficient, multiplied by -1 if the degree
        // is odd has the same sign as the limes at negative infinity
      
      f1=f[0][f[0].getDegree()];
      if ((f[0].getDegree() & 0x1) != 0)
        f1*=-1;
      
      for (int k=1; k<n; k++)
      {
        f2=f[k][f[k].getDegree()];
        if ((f[k].getDegree() & 0x1) != 0)
          f2*=-1;
        
        if ((f1 < 0 && f2 >= 0) || (f1 >= 0 && f2 < 0))
          ret++;
        
        f1=f2;
      }
      
      return ret;
    }
    
    /**
     * Number of roots in (a, b], i.e. a < x <= b. a and b must be a <= b.
     */
    
    int countRoots(T a, T b) const
    {
      if (b < a)
        swap(a, b);
      
      return countSignChanges(a)-countSignChanges(b);
    }
};

typedef SturmChain<double> SturmChaind;

}

#endif
