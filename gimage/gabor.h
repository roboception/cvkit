/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2024 Roboception GmbH
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

#ifndef GIMAGE_GABOR_H
#define GIMAGE_GABOR_H

#include "image.h"
#include "gmath/linalg.h"

namespace gimage
{

class GaborKernel
{
  public:

    GaborKernel() {}

    GaborKernel(float s, float l, float t, float g=1, float p=0)
    {
      set(s, l, t, g, p);
    }

    /**
      Creates a kernel for the Gabor transformation.

      @param s Standard deviation of Gauss function.
      @param l Wavelength of sine function.
      @param t Orientation in radians.
      @param g Aspect ratio.
      @param p Phase shift.
    */

    void set(float s, float l, float t, float g=1, float p=0)
    {
      s=std::max(0.5f, s);

      int kn=static_cast<int>(5*s+0.5f);

      if ((kn&1) == 0)
      {
        kn++;
      }

      re.setSize(kn, kn, 1);
      im.setSize(kn, kn, 1);

      int w2=re.getWidth()>>1;
      int h2=re.getHeight()>>1;

      // compute kernels for real and imaginary part

      float re_sum=0;
      float im_sum=0;

      float *rep=re.getPtr(0, 0, 0);
      float *imp=im.getPtr(0, 0, 0);

      for (long k=0; k<re.getHeight(); k++)
      {
        for (long i=0; i<re.getWidth(); i++)
        {
          double st=std::sin(t);
          double ct=std::cos(t);

          double a=(i-w2)*ct+(k-h2)*st;
          double b=-(i-w2)*st+(k-h2)*ct;

          double gs=std::exp(-(a*a+g*g*b*b)/(2*s*s));

          a=2*gmath::pi*a/l+p;

          *rep=static_cast<float>(gs*std::cos(a));
          *imp=static_cast<float>(gs*std::sin(a));

          re_sum+=std::abs(*rep++);
          im_sum+=std::abs(*imp++);
        }
      }

      // normalization of kernal to absolute sum of 1

      rep=re.getPtr(0, 0, 0);
      imp=im.getPtr(0, 0, 0);

      for (long k=0; k<re.getHeight(); k++)
      {
        for (long i=0; i<re.getWidth(); i++)
        {
          *rep++/=re_sum;
          *imp++/=im_sum;
        }
      }
    }

    const ImageFloat &get() const { return re; }
    const ImageFloat &getImaginary() const { return im; }

    int getWidth() const { return re.getWidth(); }
    int getHeight() const { return im.getHeight(); }

    float getReal(int i, int k) const { return re.get(i, k, 0); }
    float getImaginary(int i, int k) const { return im.get(i, k, 0); }

  private:

    ImageFloat re;
    ImageFloat im;
};

}

#endif
