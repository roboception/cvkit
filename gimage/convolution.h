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

#ifndef GIMAGE_CONVOLUTION_H
#define GIMAGE_CONVOLUTION_H

#include "image.h"

namespace gimage
{

template<class T, class S, class K> void convolve(Image<T> &target, const Image<S> &source,
  const Image<K> &kernel)
{
  // perform 2D convolution

  target.setSize(source.getWidth(), source.getHeight(), source.getDepth());

  long kw2=kernel.getWidth()/2;
  long kh2=kernel.getHeight()/2;

  for (int j=0; j<target.getDepth(); j++)
  {
    for (long k=0; k<target.getHeight(); k++)
    {
      for (long i=0; i<target.getWidth(); i++)
      {
        typename Image<K>::work_t v=0;

        if (k >= kh2 && k < target.getHeight()-kh2 && i >= kw2 && i < target.getWidth()-kw2)
        {
          // faster computation with less checks for inner image part

          for (long kk=0; kk<kernel.getHeight(); kk++)
          {
            S *sp=source.getPtr(i-kw2, k-kh2+kk, j);
            K *kp=kernel.getPtr(0, kk, 0);

            for (long ii=0; ii<kernel.getWidth(); ii++)
            {
              v+=static_cast<K>(*kp++ * *sp++);
            }
          }
        }
        else
        {
          // treating image border

          for (long kk=0; kk<kernel.getHeight(); kk++)
          {
            long ks=k-kh2+kk;
            for (long ii=0; ii<kernel.getWidth(); ii++)
            {
              long is=i-kw2+ii;
              v+=static_cast<K>(kernel.get(ii, kk, 0)*source.getBounds(is, ks, j));
            }
          }
        }

        // storing result

        target.setLimited(i, k, j, static_cast<typename Image<T>::work_t>(v));
      }
    }
  }
}

}

#endif
