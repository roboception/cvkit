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

#ifndef GIMAGE_COMPARE_H
#define GIMAGE_COMPARE_H

#include "image.h"

#include <vector>

namespace gimage
{

/**
  Computes an absolute difference between two images and returns the number
  of values that exceed the given tolerances. Tolerances can be given for each
  color channel separately. The last value is used if there are more color
  channels than values in tol. If tol is empty, then 0 is assumed for all
  channels. -1 is returned if the images differ in size or number of color
  channels.
*/

template<class T> long cmp(Image<T> &diff, const Image<T> &im1,
  const Image<T> &im2, const std::vector<T> &tol)
{
    if (im1.getWidth() != im2.getWidth() || im1.getHeight() != im2.getHeight() ||
      im1.getDepth() != im2.getDepth())
      return -1;
    
    diff.setSize(im1.getWidth(), im1.getHeight(), im1.getDepth());
    
    long ret=0;
    typename Image<T>::work_t t=0;
    
    for (int d=0; d<im1.getDepth(); d++)
    {
      if (d < static_cast<int>(tol.size()))
        t=tol[d];
      
      for (long k=0; k<im1.getHeight(); k++)
      {
        for (long i=0; i<im1.getWidth(); i++)
        {
          typename Image<T>::work_t v1=im1.getW(i, k, d);
          typename Image<T>::work_t v2=im2.getW(i, k, d);
          typename Image<T>::work_t v;
          
          if (im1.isValidW(v1) && im2.isValidW(v2))
            v=v1-v2;
          else if (!im1.isValidW(v1) && !im2.isValidW(v2))
            v=0;
          else
            v=Image<T>::ptraits::invalid();
          
          if (v < 0)
            v=-v;
          
          if (v > t)
            ret++;
          
          diff.setLimited(i, k, d, v);
        }
      }
    }
    
    return ret;
}

}

#endif
