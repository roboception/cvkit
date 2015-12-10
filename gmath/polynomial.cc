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

#include "polynomial.h"
#include "sturmchain.h"

#include <cmath>

#include <iostream>

using std::vector;
using std::max;
using std::abs;

namespace gmath
{

namespace {

double calcRoot(const Polynomiald &p, double low, double high)
{
    int    count=0;
    int    max=200;
    double x, xv;
    double ret;
    
      // get signs at low and high bounds
    
    double lv=p(low);
    double hv=p(high);
    
      // the lower bound is exclusive and the higher bound inclusive,
      // thus, use a higher lower bound in case of lv == 0
    
    if (lv == 0)
    {
      count=0;
      x=(low+high)/2;
      xv=p(x);
      while (xv*hv > 0 && count < max)
      {
        count++;
        x=(low+x)/2;
        xv=p(x);
      }
      
      low=x;
      lv=xv;
    }
    
      // the signs must be oposite
    
    if (lv*hv < 0)
    {
        // reduce the interval, until a midpoint between the low and high
        // boundary can not be determined
      
      count=0;
      x=(low+high)/2;
      while (x > low && x < high && count < max)
      {
        count++;
        
        xv=p(x);
        
        if (xv == 0)
          break;
        
        if (lv*xv < 0)
          high=x;
        else
          low=x;
        
        x=(high+low)/2;
      }
      
      ret=x;
    }
    
      // if the signs are not opposit, then assume that the root is exactly
      // at the low or high bound (with the Sturm method it can only be the
      // high bound, but both are checked to be on the safe side)
    
    else
    {
      if (abs(lv) < abs(hv))
        ret=low;
      else
        ret=high;
    }
    
    return ret;
}

void findRoots(vector<double> &root, const SturmChaind &s, double low,
  double high, int ln, int hn)
{
      // if there are multiple roots in the interval, try to reduce and split
      // it using the Sturm chain and recall findRoots() on partial intervals
    
    if (ln-hn > 1)
    {
      bool loop=true;
      while (loop)
      {
        loop=false;
        
        double v=(high+low)/2;
        
        if (v > low && v < high)
        {
          int vn=s.countSignChanges(v);
          
          if (vn < ln && vn > hn)
          {
            findRoots(root, s, low, v, ln, vn);
            findRoots(root, s, v, high, vn, hn);
          }
          else if (vn == ln)
          {
            low=v;
            loop=true;
          }
          else
          {
            high=v;
            loop=true;
          }
        }
        else
        {
            // this can only happen due to limited floating point resolution,
            // v is used as all roots
          
          for (int i=ln; i<hn-1; i++)
            root.push_back(v);
        }
      }
    }
      // if there is exacly one root in the interval, use bisection to
      // determine it
    
    else if (ln-hn == 1)
      root.push_back(calcRoot(s[0], low, high));
}

}

vector<double> realRoots(const Polynomiald &p)
{
    vector<double> ret;
    
      // call direct solution, if the degree is <= 3
    
    if (p.getDegree() == 1) // compute solution to first degree in closed form
    {
      ret.push_back(-p[0]/p[1]);
    }
    else if (p.getDegree() == 2) // compute solution to second degree in closed form
    {
      double v;
      
      v=p[1]*p[1]-4*p[2]*p[0];
      
      if (v > 0)
      {
        v=sqrt(v);
        
        ret.push_back((-p[1]+v)/(2*p[2]));
        ret.push_back((-p[1]-v)/(2*p[2]));
      }
      else if (v == 0)
        ret.push_back(-p[1]/(2*p[2]));
    }
    else if (p.getDegree() > 2) // compute solutions of higher degrees using Sturm chains
    {
      SturmChaind s(p);
      int ln=s.countSignChangesNegInf();
      int hn=s.countSignChangesPosInf();
      
      if (ln-hn > 0)
      {
          // determine radius around the origin that contains all roots using
          // Gerschgorins theorem
        
        double r=0;
        for (int i=p.getDegree()-1; i>=0; i--)
          r+=abs(p[i]);
        
        r/=abs(p[p.getDegree()]);
        r=max(1.0, r);
        r+=r*1e-12;
        
          // recursively determine all roots
        
        findRoots(ret, s, -r, r, ln, hn);
      }
    }
    
    return ret;
}

}
