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

#ifndef GIMAGE_LAPLACE_PYRAMID_H
#define GIMAGE_LAPLACE_PYRAMID_H

#include "image.h"
#include "gauss_pyramid.h"

#include <vector>

namespace gimage
{

/**
  Create a Laplacian pyramid. The images are downsampled by factor 2 for the
  next level.

  See Burt, P., and Adelson, E. H., "The Laplacian Pyramid as a Compact Image
  Code", IEEE Transactions on Communication, COM-31:532-540 (1983).

  @param p     Laplacian pyramid to be returned.
  @param image Input image.
*/

template<class T> void createLaplacianPyramid(std::vector<ImageFloat> &p,
  const Image<T> &image)
{
  // determine number of levels

  int n=std::min(image.getWidth(), image.getHeight());

  if (n >= 2)
  {
    n=static_cast<int>(std::floor(std::log(n)/std::log(2)));
  }
  else
  {
    n=0;
  }

  p.resize(n);

  if (n == 0) return; // input image is too small

  // populate highest level with given image

  p[0].setImage(image);

  // for all other levels

  for (int l=0; l<n-1; l++)
  {
    // compute downscaled Gaussian image for next level

    reduceGauss(p[l+1], p[l]);

    // compute Laplacian as difference of Gaussian

    const long w=p[l].getWidth();
    const long h=p[l].getHeight();

    // for all color channels

    for (int d=0; d<p[l].getDepth(); d++)
    {
      float *px=p[l].getPtr(0, 0, d);
      for (long k=0; k<2 && k<h; k++)
      {
        for (long i=0; i<w; i++)
        {
          *px++-=expandGaussPixel(p[l+1], i, k, d);
        }
      }

      for (long k=2; k<h-2; k++)
      {
        if (w > 0) *px++-=expandGaussPixel(p[l+1], 0, k, d);
        if (w > 1) *px++-=expandGaussPixel(p[l+1], 1, k, d);

        for (long i=2; i<w-2; i++)
        {
          *px++-=expandGaussPixelInner(p[l+1], i, k, d);
        }

        if (w > 2) *px++-=expandGaussPixel(p[l+1], w-2, k, d);
        if (w > 3) *px++-=expandGaussPixel(p[l+1], w-1, k, d);
      }

      for (long k=std::max(0l, h-2); k<h; k++)
      {
        for (long i=0; i<w; i++)
        {
          *px++-=expandGaussPixel(p[l+1], i, k, d);
        }
      }
    }
  }
}

template<class T> void convertCollapsedImage(Image<T> &image, const ImageFloat &p)
{
  image.setImageLimited(p);
}

template<>
inline void convertCollapsedImage(ImageU8 &image, const ImageFloat &p)
{
  // compute histogram with range -128 to 255+128

  std::vector<int> hist(512, 0);

  for (long k=0; k<p.getHeight(); k++)
  {
    for (long i=0; i<p.getWidth(); i++)
    {
      for (int d=0; d<p.getDepth(); d++)
      {
        int v=static_cast<int>(p.get(i, k, d))+128;

        if (v < 0) v=0;
        if (v > 511) v=511;

        hist[v]++;
      }
    }
  }

  // determine imin / imax value with small percentile

  int vp=std::min(100, static_cast<int>(p.getWidth()*p.getHeight()/1000));

  int sum=0;
  int imin=0;
  while (imin < 511 && sum+hist[imin] < vp) sum+=hist[imin++];

  sum=0;
  int imax=511;
  while (imax > imin && sum+hist[imax] < vp) sum+=hist[imax--];

  imin-=128;
  imax-=128;

  // bias imin / imax towards 0 / 255

  if (imax-imin < 255)
  {
    if (imin > 0) imin=std::max(0, imax-255);
    if (imax < 255) imax=std::min(255, imin+255);
  }

  // map imin / imax to 0 / 255

  image.setSize(p.getWidth(), p.getHeight(), p.getDepth());

  float offset=static_cast<float>(imin);
  float scale=255.0f/(imax-imin);

  for (long k=0; k<p.getHeight(); k++)
  {
    for (long i=0; i<p.getWidth(); i++)
    {
      for (int d=0; d<p.getDepth(); d++)
      {
        int v=static_cast<int>(scale*(p.get(i, k, d)-offset));

        if (v < 0) v=0;
        if (v > 255) v=255;

        image.set(i, k, d, static_cast<gutil::uint8>(v));
      }
    }
  }
}

/**
  Collapse a Laplacian pyramid into an image.

  See Burt, P., and Adelson, E. H., "The Laplacian Pyramid as a Compact Image
  Code", IEEE Transactions on Communication, COM-31:532-540 (1983).

  @param image Output image.
  @param p     Laplacian pyramid as input, which will be modified!
*/

template<class T> void collapseLaplacianPyramid(Image<T> &image, std::vector<ImageFloat> &p)
{
  if (p.size() < 1) return; // there is no pyramid

  if (p.size() >= 2) // there must be at least two levels for collapsing
  {
    // collapse from highest to lowest level

    for (int l=static_cast<int>(p.size())-2; l>=0; l--)
    {
      // add expanded image of lower level

      const long w=p[l].getWidth();
      const long h=p[l].getHeight();

      // for all color channels

      for (int d=0; d<p[l].getDepth(); d++)
      {
        float *px=p[l].getPtr(0, 0, d);
        for (long k=0; k<2 && k<h; k++)
        {
          for (long i=0; i<w; i++)
          {
            *px++ +=expandGaussPixel(p[l+1], i, k, d);
          }
        }

        for (long k=2; k<h-2; k++)
        {
          if (w > 0) *px++ +=expandGaussPixel(p[l+1], 0, k, d);
          if (w > 1) *px++ +=expandGaussPixel(p[l+1], 1, k, d);

          for (long i=2; i<w-2; i++)
          {
            *px++ +=expandGaussPixelInner(p[l+1], i, k, d);
          }

          if (w > 2) *px++ +=expandGaussPixel(p[l+1], w-2, k, d);
          if (w > 3) *px++ +=expandGaussPixel(p[l+1], w-1, k, d);
        }

        for (long k=std::max(0l, h-2); k<h; k++)
        {
          for (long i=0; i<w; i++)
          {
            *px++ +=expandGaussPixel(p[l+1], i, k, d);
          }
        }
      }
    }
  }

  // image at level 0 is result

  convertCollapsedImage(image, p[0]);
}

}

#endif
