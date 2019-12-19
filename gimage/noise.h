/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2019 Roboception GmbH
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

#ifndef GIMAGE_NOISE_H
#define GIMAGE_NOISE_H

#include "image.h"

#include <algorithm>
#include <cstdlib>

namespace gimage
{

inline void getBoxMuller(double &ret1, double &ret2, double stddev)
{
  double u, v, s;

  do
  {
    u=(2.0*rand())/RAND_MAX-1.0;
    v=(2.0*rand())/RAND_MAX-1.0;
    s=u*u+v*v;
  }
  while (s == 0 || s >= 1.0);

  s=sqrt(-2.0*log(s)/s);

  ret1=u*s*stddev;
  ret2=v*s*stddev;
}

/**
  Adds Gaussian noise with the given standard deviation to all pixel values.
*/

template<class T> void addNoise(Image<T> &image, float stddev)
{
  double vmin=image.absMinValue();
  double vmax=image.absMaxValue();

  for (long d=0; d<image.getDepth(); d++)
  {
    for (long k=0; k<image.getHeight(); k++)
    {
      for (long i=0; i<image.getWidth(); i+=2)
      {
        double r1, r2;
        getBoxMuller(r1, r2, stddev);

        double v=std::max(vmin, std::min(vmax, image.get(i, k, d)+r1));
        image.set(i, k, d, static_cast<typename Image<T>::store_t>(v));

        if (i+1 < image.getWidth())
        {
          v=std::max(vmin, std::min(vmax, image.get(i+1, k, d)+r2));
          image.set(i+1, k, d, static_cast<typename Image<T>::store_t>(v));
        }
      }
    }
  }
}

/**
  Adds Gaussian noise scaled to the pixel intensity to all pixel values. This
  mimics the behavior of real cameras.
*/

template<class T> void addScaledNoise(Image<T> &image, float stddev)
{
  double vmin=image.absMinValue();
  double vmax=image.absMaxValue();

  for (long d=0; d<image.getDepth(); d++)
  {
    for (long k=0; k<image.getHeight(); k++)
    {
      for (long i=0; i<image.getWidth(); i+=2)
      {
        double r1, r2;
        getBoxMuller(r1, r2, stddev);

        double v=image.get(i, k, d);
        v=std::max(vmin, std::min(vmax, v+r1*v/vmax));
        image.set(i, k, d, static_cast<typename Image<T>::store_t>(v));

        if (i+1 < image.getWidth())
        {
          v=image.get(i+1, k, d);
          v=std::max(vmin, std::min(vmax, v+r2*v/vmax));
          image.set(i+1, k, d, static_cast<typename Image<T>::store_t>(v));
        }
      }
    }
  }
}

}

#endif
