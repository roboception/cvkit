/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2021 Roboception GmbH
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

#ifndef GIMAGE_GAUSS_H
#define GIMAGE_GAUSS_H

#include "image.h"

#include <vector>

namespace gimage
{

class GaussKernel
{
  public:

    GaussKernel()
    {
      kernel=0;
      kn=0;
      kn2=0;
    }

    GaussKernel(float s)
    {
      kernel=0;
      kn=0;
      kn2=0;

      set(s);
    }

    void set(float s)
    {
      s=std::max(0.5f, s);

      kn=static_cast<int>(5*s+0.5f);

      if ((kn&1) == 0)
      {
        kn++;
      }

      kernel=new float [kn];

      kn2=kn>>1;

      float sum=0;
      for (int i=0; i<kn; i++)
      {
        kernel[i]=static_cast<float>(std::exp(-(i-kn2)*(i-kn2)/(2*s*s)));
        sum+=kernel[i];
      }

      for (int i=0; i<kn; i++)
      {
        kernel[i]/=sum;
      }
    }

    ~GaussKernel()
    {
      delete [] kernel;
    }

    ImageFloat get() const
    {
      ImageFloat ret(kn, kn, 1);

      for (int k=0; k<kn; k++)
      {
        for (int i=0; i<kn; i++)
        {
          ret.set(i, k, 0, kernel[k]*kernel[i]);
        }
      }

      return ret;
    }

    template<class T> inline float convolveHorizontal(const Image<T> &image,
        long i, long k, int d) const
    {
      if (i >= kn2 && i < image.getWidth()-kn2)
      {
        float pv=0;

        int jj=i-kn2;
        for (long j=0; j<kn; j++)
        {
          pv+=kernel[j]*image.get(jj++, k, d);
        }

        return pv;
      }
      else
      {
        float pv=0;
        float ps=0;

        for (long j=0; j<kn; j++)
        {
          int jj=i-kn2+j;

          if (jj >= 0 && jj < image.getWidth())
          {
            pv+=kernel[j]*image.get(jj, k, d);
            ps+=kernel[j];
          }
        }

        return pv/ps;
      }
    }

    template<class T> inline float convolveVertical(const Image<T> &image,
        long i, long k, int d) const
    {
      if (k >= kn2 && k < image.getHeight()-kn2)
      {
        float pv=0;

        int jj=k-kn2;
        for (long j=0; j<kn; j++)
        {
          pv+=kernel[j]*image.get(i, jj++, d);
        }

        return pv;
      }
      else
      {
        float pv=0;
        float ps=0;

        for (long j=0; j<kn; j++)
        {
          int jj=k-kn2+j;

          if (jj >= 0 && jj < image.getHeight())
          {
            pv+=kernel[j]*image.get(i, jj, d);
            ps+=kernel[j];
          }
        }

        return pv/ps;
      }
    }

  private:

    GaussKernel(const GaussKernel &);

    float *kernel;
    int kn;
    int kn2;
};

/**
  Performs Gaussian smoothing on the given image.

  Note: This is faster than using the Gauss kernel for a general 2D
  convolution.

  @param image Input / output image.
  @param s     Standard deviation.
*/

template<class T> void gauss(Image<T> &image, float s)
{
  GaussKernel kernel(s);

  ImageFloat tmp(std::max(image.getWidth(), image.getHeight()), 1, 1);
  float *p=tmp.getPtr(0, 0, 0);

  // for all color channels

  for (int d=0; d<image.getDepth(); d++)
  {
    // convolve horizontal in place

    for (long k=0; k<image.getHeight(); k++)
    {
      for (long i=0; i<image.getWidth(); i++)
      {
        p[i]=kernel.convolveHorizontal(image, i, k, d);
      }

      T *tp=image.getPtr(0, k, d);
      for (long i=0; i<image.getWidth(); i++)
      {
        *tp++=static_cast<T>(p[i]);
      }
    }

    // convolve vertically in place

    for (long i=0; i<image.getWidth(); i++)
    {
      for (long k=0; k<image.getHeight(); k++)
      {
        p[k]=kernel.convolveVertical(image, i, k, d);
      }

      T *tp=image.getPtr(i, 0, d);
      for (long k=0; k<image.getHeight(); k++)
      {
        *tp=static_cast<T>(p[k]);
        tp+=image.getWidth();
      }
    }
  }
}

/**
  Performs Gaussian smoothing without changing the source image.

  Note: This is faster than using the Gauss kernel for a general 2D
  convolution.

  @param target Output image.
  @param image  Input image.
  @param s      Standard deviation.
*/

template<class T> void gauss(Image<T> &target, const Image<T> &image, float s)
{
  GaussKernel kernel(s);

  ImageFloat tmp(image.getHeight(), 1, 1);
  float *p=tmp.getPtr(0, 0, 0);

  target.setSize(image.getWidth(), image.getHeight(), image.getDepth());

  // for all color channels

  for (int d=0; d<image.getDepth(); d++)
  {
    // convolve horizontal into target image

    T *tp=target.getPtr(0, 0, d);
    for (long k=0; k<image.getHeight(); k++)
    {
      for (long i=0; i<image.getWidth(); i++)
      {
        *tp++=static_cast<T>(kernel.convolveHorizontal(image, i, k, d));
      }
    }

    // convolve vertically in place

    for (long i=0; i<image.getWidth(); i++)
    {
      for (long k=0; k<image.getHeight(); k++)
      {
        p[k]=kernel.convolveVertical(target, i, k, d);
      }

      tp=target.getPtr(i, 0, d);
      for (long k=0; k<image.getHeight(); k++)
      {
        *tp=static_cast<T>(p[k]);
        tp+=target.getWidth();
      }
    }
  }
}

}

#endif
