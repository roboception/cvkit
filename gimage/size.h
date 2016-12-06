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

#include <gutil/proctime.h>

#include <vector>

namespace gimage
{

template<class T> Image<T> downscaleImage(const Image<T> &image, int factor)
{
    factor=std::max(1, factor);

    if (factor == 1)
      return image;

    Image<T> ret((image.getWidth()+factor-1)/factor,
      (image.getHeight()+factor-1)/factor, image.getDepth());

    for (int d=0; d<image.getDepth(); d++)
    {
      T *out=ret.getPtr(0, 0, d);

      long k=0;
      while (k+factor <= image.getHeight())
      {
        long i=0;

        if (factor == 2)
        {
          // speeding up special case of averaging over 2*2 pixels

          while (i+2 <= image.getWidth())
          {
            typename Image<T>::work_t v;

            const T *in=image.getPtr(i, k, d);

            v=*in+in[1];
            in+=image.getWidth();
            v+=*in+in[1];

            *out++=static_cast<typename Image<T>::store_t>((v+2)/4);
            i+=2;
          }
        }
        else if (factor == 3)
        {
          // speeding up special case of averaging over 3*3 pixels

          while (i+3 <= image.getWidth())
          {
            typename Image<T>::work_t v=0;

            const T *in=image.getPtr(i, k, d);

            for (int j=0; j<3; j++)
            {
              v+=*in+in[1]+in[2];
              in+=image.getWidth();
            }

            *out++=static_cast<typename Image<T>::store_t>((v+4)/9);
            i+=3;
          }
        }
        else if (factor == 4)
        {
          // speeding up special case of averaging over 4*4 pixels

          while (i+4 <= image.getWidth())
          {
            typename Image<T>::work_t v=0;

            const T *in=image.getPtr(i, k, d);

            for (int j=0; j<4; j++)
            {
              v+=*in+in[1]+in[2]+in[3];
              in+=image.getWidth();
            }

            *out++=static_cast<typename Image<T>::store_t>((v+8)/16);
            i+=4;
          }
        }
        else
        {
          // average over factor*factor pixels of the input image

          while (i+factor <= image.getWidth())
          {
            typename Image<T>::work_t v=0;
            int n=0;

            const T *in=image.getPtr(i, k, d);
            for (int kk=0; kk<factor; kk++)
            {
              for (int ii=0; ii<factor; ii++)
              {
                v+=in[ii];
                n++;
              }

              in+=image.getWidth();
            }

            *out++=static_cast<typename Image<T>::store_t>((v+(n>>1))/n);
            i+=factor;
          }
        }

        // if there are less than factor pixels left in the image row, then
        // average with boundary check

        if (i < image.getWidth())
        {
          typename Image<T>::work_t v=0;
          int n=0;

          const T *in=image.getPtr(i, k, d);
          for (int kk=0; kk<factor; kk++)
          {
            for (int ii=0; ii<factor && i+ii<image.getWidth(); ii++)
            {
              v+=in[ii];
              n++;
            }

            in+=image.getWidth();
          }

          *out++=static_cast<typename Image<T>::store_t>((v+(n>>1))/n);
        }

        k+=factor;
      }

      // if there are less than factor image rows left in the image, then
      // average with boundary check

      if (k < image.getHeight())
      {
        for (long i=0; i<image.getWidth(); i+=factor)
        {
          typename Image<T>::work_t v=0;
          int n=0;

          const T *in=image.getPtr(i, k, d);
          for (int kk=0; kk<factor && k+kk<image.getHeight(); kk++)
          {
            for (int ii=0; ii<factor && i+ii<image.getWidth(); ii++)
            {
              v+=in[ii];
              n++;
            }

            in+=image.getWidth();
          }

          *out++=static_cast<typename Image<T>::store_t>((v+(n>>1))/n);
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
