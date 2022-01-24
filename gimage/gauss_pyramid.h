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

#ifndef GIMAGE_GAUSS_PYRAMID_H
#define GIMAGE_GAUSS_PYRAMID_H

#include "image.h"

namespace gimage
{

/**
  Performs Gaussian smoothing and downscaling by factor 2.

  See Burt, P., and Adelson, E. H., "The Laplacian Pyramid as a Compact Image
  Code", IEEE Transactions on Communication, COM-31:532-540 (1983).

  @param target Output image.
  @param image  Input image.
*/

template<class T, class S> void reduceGauss(Image<T> &target, const Image<S> &image)
{
  static const float g[]={0.0625, 0.25, 0.375, 0.25, 0.0625};

  // intermediate image with downscaled width

  ImageFloat tmp((image.getWidth()+1)/2, image.getHeight(), 1);

  // downscaled target image

  target.setSize((image.getWidth()+1)/2, (image.getHeight()+1)/2, image.getDepth());

  // for all color channels

  for (int d=0; d<image.getDepth(); d++)
  {
    // convolve horizontal into horizontally downscaled intermediate image

    float *tmpp=tmp.getPtr(0, 0, 0);
    for (long k=0; k<image.getHeight(); k++)
    {
      for (long i=0; i<image.getWidth(); i+=2)
      {
        float pv=0;

        if (i >= 2 && i < image.getWidth()-2)
        {
          S *p=image.getPtr(i-2, k, d);
          for (long j=0; j<5; j++)
          {
            pv+=g[j]* *p++;
          }
        }
        else
        {
          for (long j=0; j<5; j++)
          {
            pv+=g[j]*image.getBounds(i+j-2, k, d);
          }
        }

        *tmpp++=pv;
      }
    }

    // convolve vertically into downscaled target image

    for (long i=0; i<tmp.getWidth(); i++)
    {
      T *tp=target.getPtr(i, 0, d);
      for (long k=0; k<tmp.getHeight(); k+=2)
      {
        float pv=0;

        if (k >= 2 && k < tmp.getHeight()-2)
        {
          float *p=tmp.getPtr(i, k-2, 0);
          for (long j=0; j<5; j++)
          {
            pv+=g[j]* *p;
            p+=tmp.getWidth();
          }
        }
        else
        {
          for (long j=0; j<5; j++)
          {
            pv+=g[j]*tmp.getBounds(i, k+j-2, 0);
          }
        }

        *tp=static_cast<T>(pv);
        tp+=target.getWidth();
      }
    }
  }
}

/**
  Create a Gaussian pyramid. The image is first smoothed with Gaussian and then
  downsampled by factor 2 for the next level.

  See Burt, P., and Adelson, E. H., "The Laplacian Pyramid as a Compact Image
  Code", IEEE Transactions on Communication, COM-31:532-540 (1983).

  @param p     Gaussian pyramid to be returned.
  @param image Input image
  @param s     Standard deviation for Gaussian smoothing
*/

template<class T, class S> void createGaussPyramid(std::vector<Image<T> > &p, const Image<S> &image)
{
  // determine number of levels

  int n=std::min(image.getWidth(), image.getHeight());

  if (n >= 2)
  {
    n=std::floor(std::log(n)/std::log(2));
  }
  else
  {
    n=0;
  }

  p.resize(n);

  if (n == 0) return; // input image is too small

  // populate highest level with given image if not already done

  if (&(p[0]) != &image)
  {
    p[0].setImage(image);
  }

  // smooth and downsample image for all other levels

  for (int i=1; i<n; i++)
  {
    reduceGauss(p[i], p[i-1]);
  }
}

/**
  Returns the pixel value of the image that is created by Gaussian upscaling
  of the given image. The size of the upscaled image is twice as the size of
  the given image.

  See Burt, P., and Adelson, E. H., "The Laplacian Pyramid as a Compact Image
  Code", IEEE Transactions on Communication, COM-31:532-540 (1983).

  @param image Input image.
  @param i     Column of upscaled image, with i < 2*image.getWidth().
  @param k     Row of upscaled image, with k < 2*image.getHeight().
  @param j     Index of color channel of input image.
  @return      Pixel value at i, k, in the upscaled image.
*/

template<class T> float expandGaussPixel(const Image<T> &image, long i, long k, int j)
{
  static const float g[5][5]=
  {
    { 0.0039062, 0.0156250, 0.0234375, 0.0156250, 0.0039062},
    { 0.0156250, 0.0625000, 0.0937500, 0.0625000, 0.0156250},
    { 0.0234375, 0.0937500, 0.1406250, 0.0937500, 0.0234375},
    { 0.0156250, 0.0625000, 0.0937500, 0.0625000, 0.0156250},
    { 0.0039062, 0.0156250, 0.0234375, 0.0156250, 0.0039062},
  };

  float ret;

  if (i&0x1)
  {
    if (k&0x1)
    {
      long ii=i>>1;
      long kk=k>>1;

      ret=(image.getBounds(ii, kk, j)+image.getBounds(ii+1, kk, j)+
           image.getBounds(ii, kk+1, j)+image.getBounds(ii+1, kk+1, j))/4;
    }
    else
    {
      long ii=i>>1;
      long kk=(k>>1)-1;

      ret =g[0][1]*image.getBounds(ii, kk, j)+g[0][3]*image.getBounds(ii+1, kk, j);
      ret+=g[2][1]*image.getBounds(ii, kk+1, j)+g[2][3]*image.getBounds(ii+1, kk+1, j);
      ret+=g[4][1]*image.getBounds(ii, kk+2, j)+g[4][3]*image.getBounds(ii+1, kk+2, j);
      ret*=4;
    }
  }
  else
  {
    if (k&0x1)
    {
      long ii=(i>>1)-1;
      long kk=k>>1;

      ret =g[1][0]*image.getBounds(ii, kk, j)+g[1][2]*image.getBounds(ii+1, kk, j)+g[1][4]*image.getBounds(ii+2, kk, j);
      ret+=g[3][0]*image.getBounds(ii, kk+1, j)+g[3][2]*image.getBounds(ii+1, kk+1, j)+g[3][4]*image.getBounds(ii+2, kk+1, j);
      ret*=4;
    }
    else
    {
      long ii=(i>>1)-1;
      long kk=(k>>1)-1;

      ret =g[0][0]*image.getBounds(ii, kk, j)+g[0][2]*image.getBounds(ii+1, kk, j)+g[0][4]*image.getBounds(ii+2, kk, j);
      ret+=g[2][0]*image.getBounds(ii, kk+1, j)+g[2][2]*image.getBounds(ii+1, kk+1, j)+g[2][4]*image.getBounds(ii+2, kk+1, j);
      ret+=g[4][0]*image.getBounds(ii, kk+2, j)+g[4][2]*image.getBounds(ii+1, kk+2, j)+g[4][4]*image.getBounds(ii+2, kk+2, j);
      ret*=4;
    }
  }

  return ret;
}

/**
  Same as expandGaussPixel(), but much faster because there is not image border
  checking. This means that only i and k are allowed for which:

  2 <= i < 2*image.getWidth()-2 and 2 <= k < 2*image.getHeight()-2

  Otherwise, the function will crash.
*/

template<class T> float expandGaussPixelInner(const Image<T> &image, long i, long k, int j)
{
  static const float g[5][5]=
  {
    { 0.0039062, 0.0156250, 0.0234375, 0.0156250, 0.0039062},
    { 0.0156250, 0.0625000, 0.0937500, 0.0625000, 0.0156250},
    { 0.0234375, 0.0937500, 0.1406250, 0.0937500, 0.0234375},
    { 0.0156250, 0.0625000, 0.0937500, 0.0625000, 0.0156250},
    { 0.0039062, 0.0156250, 0.0234375, 0.0156250, 0.0039062},
  };

  const long w=image.getWidth();
  float ret;

  if (i&0x1)
  {
    if (k&0x1)
    {
      T *p=image.getPtr(i>>1, k>>1, j);

      ret=(p[0]+p[1]+p[w]+p[w+1])/4;
    }
    else
    {
      T *p=image.getPtr(i>>1, (k>>1)-1, j);

      ret =g[0][1]*p[0]+g[0][3]*p[1]; p+=w;
      ret+=g[2][1]*p[0]+g[2][3]*p[1]; p+=w;
      ret+=g[4][1]*p[0]+g[4][3]*p[1];
      ret*=4;
    }
  }
  else
  {
    if (k&0x1)
    {
      T *p=image.getPtr((i>>1)-1, k>>1, j);

      ret =g[1][0]*p[0]+g[1][2]*p[1]+g[1][4]*p[2]; p+=w;
      ret+=g[3][0]*p[0]+g[3][2]*p[1]+g[3][4]*p[2];
      ret*=4;
    }
    else
    {
      T *p=image.getPtr((i>>1)-1, (k>>1)-1, j);

      ret =g[0][0]*p[0]+g[0][2]*p[1]+g[0][4]*p[2]; p+=w;
      ret+=g[2][0]*p[0]+g[2][2]*p[1]+g[2][4]*p[2]; p+=w;
      ret+=g[4][0]*p[0]+g[4][2]*p[1]+g[4][4]*p[2];
      ret*=4;
    }
  }

  return ret;
}

/**
  Performs Gaussian smoothing and upsampling by factor 2.

  See Burt, P., and Adelson, E. H., "The Laplacian Pyramid as a Compact Image
  Code", IEEE Transactions on Communication, COM-31:532-540 (1983).

  @param target Output image.
  @param image  Input image.
*/

template<class T, class S> void expandGauss(Image<T> &target, const Image<S> &image)
{
  const long w=2*image.getWidth();
  const long h=2*image.getHeight();

  target.setSize(w, h, image.getDepth());

  // for all color channels

  for (int d=0; d<target.getDepth(); d++)
  {
    T *p=target.getPtr(0, 0, d);
    for (long k=0; k<2 && k<h; k++)
    {
      for (long i=0; i<w; i++)
      {
        *p++=static_cast<T>(expandGaussPixel(image, i, k, d));
      }
    }

    for (long k=2; k<h-2; k++)
    {
      if (w > 0) *p++=static_cast<T>(expandGaussPixel(image, 0, k, d));
      if (w > 1) *p++=static_cast<T>(expandGaussPixel(image, 1, k, d));

      for (long i=2; i<w-2; i++)
      {
        *p++=static_cast<T>(expandGaussPixelInner(image, i, k, d));
      }

      if (w > 2) *p++=static_cast<T>(expandGaussPixel(image, w-2, k, d));
      if (w > 3) *p++=static_cast<T>(expandGaussPixel(image, w-1, k, d));
    }

    for (long k=std::max(0l, h-2); k<h; k++)
    {
      for (long i=0; i<w; i++)
      {
        *p++=static_cast<T>(expandGaussPixel(image, i, k, d));
      }
    }
  }
}

}

#endif
