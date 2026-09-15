/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2016 Roboception GmbH
 * Copyright (c) 2014 Institute of Robotics and Mechatronics, German Aerospace Center
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

#ifndef GIMAGE_SIZE_H
#define GIMAGE_SIZE_H

#include "image.h"

#include <vector>

namespace gimage
{

/**
 * Averages over blocks of factor*factor pixels of the given depth layer of the
 * given image and stores the result in the given image. The factor is a
 * template parameter, so that the compiler can replace the division by a
 * multiplication and unroll the inner loops, which is significantly faster
 * than passing the factor as a parameter.
 *
 * This is a helper of downscaleImage() and expects that the target image has
 * already been created with the correct size.
 */

template<class T, int F> void downscaleImageBlock(Image<T> &ret, const Image<T> &image,
    int d)
{
  const long w=image.getWidth();
  const long h=image.getHeight();

  T *out=ret.getPtr(0, 0, d);

  long k=0;

  while (k+F <= h)
  {
    const T *row=image.getPtr(0, k, d);

    // average over F*F pixels of the input image

    long i=0;

    while (i+F <= w)
    {
      typename Image<T>::work_t v=0;

      const T *in=row+i;

      for (int kk=0; kk<F; kk++)
      {
        for (int ii=0; ii<F; ii++)
        {
          v+=in[ii];
        }

        in+=w;
      }

      *out++=static_cast<typename Image<T>::store_t>((v+F*F/2)/(F*F));
      i+=F;
    }

    // if there are less than F pixels left in the image row, then average
    // with boundary check

    if (i < w)
    {
      typename Image<T>::work_t v=0;
      int n=0;

      const T *in=row+i;

      for (int kk=0; kk<F; kk++)
      {
        for (int ii=0; ii<F && i+ii<w; ii++)
        {
          v+=in[ii];
          n++;
        }

        in+=w;
      }

      *out++=static_cast<typename Image<T>::store_t>((v+(n>>1))/n);
    }

    k+=F;
  }

  // if there are less than F image rows left in the image, then average with
  // boundary check

  if (k < h)
  {
    for (long i=0; i<w; i+=F)
    {
      typename Image<T>::work_t v=0;
      int n=0;

      const T *in=image.getPtr(i, k, d);

      for (int kk=0; kk<F && k+kk<h; kk++)
      {
        for (int ii=0; ii<F && i+ii<w; ii++)
        {
          v+=in[ii];
          n++;
        }

        in+=w;
      }

      *out++=static_cast<typename Image<T>::store_t>((v+(n>>1))/n);
    }
  }
}

/**
 * Like downscaleImageBlock(), but with the factor as a parameter, for factors
 * that are too large for an own instantiation of the function above.
 */

template<class T> void downscaleImageBlockN(Image<T> &ret, const Image<T> &image, int d,
    int factor)
{
  const long w=image.getWidth();
  const long h=image.getHeight();

  T *out=ret.getPtr(0, 0, d);

  long k=0;

  while (k+factor <= h)
  {
    const T *row=image.getPtr(0, k, d);

    // average over factor*factor pixels of the input image

    long i=0;

    while (i+factor <= w)
    {
      typename Image<T>::work_t v=0;
      int n=0;

      const T *in=row+i;

      for (int kk=0; kk<factor; kk++)
      {
        for (int ii=0; ii<factor; ii++)
        {
          v+=in[ii];
          n++;
        }

        in+=w;
      }

      *out++=static_cast<typename Image<T>::store_t>((v+(n>>1))/n);
      i+=factor;
    }

    // if there are less than factor pixels left in the image row, then average
    // with boundary check

    if (i < w)
    {
      typename Image<T>::work_t v=0;
      int n=0;

      const T *in=row+i;

      for (int kk=0; kk<factor; kk++)
      {
        for (int ii=0; ii<factor && i+ii<w; ii++)
        {
          v+=in[ii];
          n++;
        }

        in+=w;
      }

      *out++=static_cast<typename Image<T>::store_t>((v+(n>>1))/n);
    }

    k+=factor;
  }

  // if there are less than factor image rows left in the image, then average
  // with boundary check

  if (k < h)
  {
    for (long i=0; i<w; i+=factor)
    {
      typename Image<T>::work_t v=0;
      int n=0;

      const T *in=image.getPtr(i, k, d);

      for (int kk=0; kk<factor && k+kk<h; kk++)
      {
        for (int ii=0; ii<factor && i+ii<w; ii++)
        {
          v+=in[ii];
          n++;
        }

        in+=w;
      }

      *out++=static_cast<typename Image<T>::store_t>((v+(n>>1))/n);
    }
  }
}

template<class T> Image<T> downscaleImage(const Image<T> &image, int factor)
{
  factor=std::max(1, factor);

  if (factor == 1)
  {
    return image;
  }

  Image<T> ret((image.getWidth()+factor-1)/factor,
               (image.getHeight()+factor-1)/factor, image.getDepth());

  for (int d=0; d<image.getDepth(); d++)
  {
    switch (factor)
    {
      case 2:
        downscaleImageBlock<T, 2>(ret, image, d);
        break;

      case 3:
        downscaleImageBlock<T, 3>(ret, image, d);
        break;

      case 4:
        downscaleImageBlock<T, 4>(ret, image, d);
        break;

      case 5:
        downscaleImageBlock<T, 5>(ret, image, d);
        break;

      case 6:
        downscaleImageBlock<T, 6>(ret, image, d);
        break;

      case 7:
        downscaleImageBlock<T, 7>(ret, image, d);
        break;

      case 8:
        downscaleImageBlock<T, 8>(ret, image, d);
        break;

      default:
        downscaleImageBlockN(ret, image, d, factor);
        break;
    }
  }

  return ret;
}

template<> inline Image<float> downscaleImage(const Image<float> &image, int factor)
{
  factor=std::max(1, factor);

  if (factor == 1)
  {
    return image;
  }

  Image<float> ret((image.getWidth()+factor-1)/factor,
                   (image.getHeight()+factor-1)/factor, image.getDepth());

  for (int d=0; d<image.getDepth(); d++)
  {
    float *out=ret.getPtr(0, 0, d);

    long k=0;

    while (k+factor <= image.getHeight())
    {
      long i=0;

      // average over factor*factor pixels of the input image

      while (i+factor <= image.getWidth())
      {
        typename Image<float>::work_t v=0;
        int n=0;

        const float *in=image.getPtr(i, k, d);

        for (int kk=0; kk<factor; kk++)
        {
          for (int ii=0; ii<factor; ii++)
          {
            if (image.isValidS(in[ii]))
            {
              v+=in[ii];
              n++;
            }
          }

          in+=image.getWidth();
        }

        if (n > 0)
        {
          *out++=static_cast<typename Image<float>::store_t>(v/n);
        }
        else
        {
          *out++=PixelTraits<float>::invalid();
        }

        i+=factor;
      }

      // if there are less than factor pixels left in the image row, then
      // average with boundary check

      if (i < image.getWidth())
      {
        typename Image<float>::work_t v=0;
        int n=0;

        const float *in=image.getPtr(i, k, d);

        for (int kk=0; kk<factor; kk++)
        {
          for (int ii=0; ii<factor && i+ii<image.getWidth(); ii++)
          {
            if (image.isValidS(in[ii]))
            {
              v+=in[ii];
              n++;
            }
          }

          in+=image.getWidth();
        }

        if (n > 0)
        {
          *out++=static_cast<typename Image<float>::store_t>(v/n);
        }
        else
        {
          *out++=PixelTraits<float>::invalid();
        }
      }

      k+=factor;
    }

    // if there are less than factor image rows left in the image, then
    // average with boundary check

    if (k < image.getHeight())
    {
      for (long i=0; i<image.getWidth(); i+=factor)
      {
        typename Image<float>::work_t v=0;
        int n=0;

        const float *in=image.getPtr(i, k, d);

        for (int kk=0; kk<factor && k+kk<image.getHeight(); kk++)
        {
          for (int ii=0; ii<factor && i+ii<image.getWidth(); ii++)
          {
            if (image.isValidS(in[ii]))
            {
              v+=in[ii];
              n++;
            }
          }

          in+=image.getWidth();
        }

        if (n > 0)
        {
          *out++=static_cast<typename Image<float>::store_t>(v/n);
        }
        else
        {
          *out++=PixelTraits<float>::invalid();
        }
      }
    }
  }

  return ret;
}

template<class T> Image<T> medianDownscaleImage(const Image<T> &image, int factor)
{
  factor=std::max(1, factor);

  Image<T> ret((image.getWidth()+factor-1)/factor,
               (image.getHeight()+factor-1)/factor, image.getDepth());

  std::vector<T> v(factor*factor, 0);

  for (int d=0; d<image.getDepth(); d++)
  {
    for (long k=0; k<image.getHeight(); k+=factor)
    {
      for (long i=0; i<image.getWidth(); i+=factor)
      {
        int n=0;

        for (int kk=0; kk<factor && k+kk<image.getHeight(); kk++)
        {
          for (int ii=0; ii<factor && i+ii<image.getWidth(); ii++)
          {
            if (image.isValid(i+ii, k+kk))
            {
              v[n]=image.get(i+ii, k+kk, d);
              n++;
            }
          }
        }

        ret.setInvalid(i/factor, k/factor, d);

        if (n > 0)
        {
          partial_sort(v.begin(), v.begin()+(n>>1), v.begin()+n);
          ret.set(i/factor, k/factor, d, v[n>>1]);
        }
      }
    }
  }

  return ret;
}

template<class T> Image<T> resizeImageBilinear(const Image<T> &image, long w, long h)
{
  Image<T> ret(w, h, image.getDepth());

  const float fx=static_cast<float>(image.getWidth())/w;
  const float fy=static_cast<float>(image.getHeight())/h;

  // getBilinear() is not called per pixel, because the addressing and the
  // weights that only depend on the column are computed once for all rows and
  // the pixel buffers are accessed directly

  if (image.getWidth() < 2 || image.getHeight() < 2)
  {
    // bilinear interpolation needs at least two rows and columns, which
    // getBilinear() handles

    std::vector<typename Image<T>::work_t> v(image.getDepth());

    for (long k=0; k<h; k++)
    {
      for (long i=0; i<w; i++)
      {
        image.getBilinear(v, (i+0.5)*fx, (k+0.5)*fy);

        for (int d=0; d<image.getDepth(); d++)
        {
          ret.set(i, k, d, static_cast<typename Image<T>::store_t>(v[d]));
        }
      }
    }

    return ret;
  }

  const typename Image<T>::store_t invalid=
    static_cast<typename Image<T>::store_t>(Image<T>::ptraits::invalid());

  // column of the left neighbor and the two weights in x direction, which are
  // scaled by 4, as in getBilinear()

  std::vector<long> sx(w);
  std::vector<float> ax0(w), ax1(w);

  for (long i=0; i<w; i++)
  {
    float x=static_cast<float>((i+0.5)*fx)-0.5f;

    if (x < 0)
    {
      x=0;
    }

    if (x >= image.getWidth()-1)
    {
      x=image.getWidth()-1.001f;
    }

    const long si=static_cast<long>(x);

    sx[i]=si;
    ax1[i]=4*(x-si);
    ax0[i]=4-ax1[i];
  }

  for (long k=0; k<h; k++)
  {
    float y=static_cast<float>((k+0.5)*fy)-0.5f;

    if (y < 0)
    {
      y=0;
    }

    if (y >= image.getHeight()-1)
    {
      y=image.getHeight()-1.001f;
    }

    const long sk=static_cast<long>(y);

    const float ay1=4*(y-sk);
    const float ay0=4-ay1;

    for (int d=0; d<image.getDepth(); d++)
    {
      const T *r0=image.getPtr(0, sk, d);
      const T *r1=image.getPtr(0, sk+1, d);

      T *out=ret.getPtr(0, k, d);

      for (long i=0; i<w; i++)
      {
        const long si=sx[i];

        const typename Image<T>::store_t p0=r0[si];
        const typename Image<T>::store_t p1=r0[si+1];
        const typename Image<T>::store_t p2=r1[si];
        const typename Image<T>::store_t p3=r1[si+1];

        out[i]=invalid;

        if (image.isValidS(p0) && image.isValidS(p1) &&
            image.isValidS(p2) && image.isValidS(p3))
        {
          const float s0=ax0[i]*ay0;
          const float s1=ax1[i]*ay0;
          const float s2=ax0[i]*ay1;
          const float s3=ax1[i]*ay1;

          out[i]=static_cast<typename Image<T>::store_t>(
            static_cast<typename Image<T>::work_t>((p0*s0+p1*s1+p2*s2+p3*s3)/16));
        }
      }
    }
  }

  return ret;
}

template<class T> Image<T> cropImage(const Image<T> &image, long x, long y, long w, long h)
{
  w=std::max(0l, w);
  h=std::max(0l, h);

  Image<T> ret(w, h, image.getDepth());

  if (x >= 0 && y >= 0 && x+w <= image.getWidth() && y+h <= image.getHeight())
  {
    for (int d=0; d<image.getDepth(); d++)
    {
      for (long k=0; k<h; k++)
      {
        memcpy(ret.getPtr(0, k, d), image.getPtr(x, y+k, d), w*sizeof(T));
      }
    }
  }
  else
  {
    for (int d=0; d<image.getDepth(); d++)
    {
      for (long k=0; k<h; k++)
      {
        for (long i=0; i<w; i++)
          ret.set(i, k, d,
                  static_cast<typename Image<T>::store_t>(image.getBoundsInv(x+i, y+k, d)));
      }
    }
  }

  return ret;
}

}

#endif
