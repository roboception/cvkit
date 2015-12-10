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

#ifndef GIMAGE_ARITHMETIC_H
#define GIMAGE_ARITHMETIC_H

#include "image.h"

#include <gutil/exception.h>

namespace gimage
{

template<class T> Image<T>& operator+=(Image<T> &image, typename Image<T>::work_t s)
{
    for (int d=0; d<image.getDepth(); d++)
    {
      for (long k=0; k<image.getHeight(); k++)
      {
        for (long i=0; i<image.getWidth(); i++)
        {
          T v=0;
          
          v=image.get(i, k, d);
          if (image.isValidS(v))
            image.setLimited(i, k, d, v+s);
        }
      }
    }
    
    return image;
}

template<class T> Image<T>& operator-=(Image<T> &image, typename Image<T>::work_t s)
{
    for (int d=0; d<image.getDepth(); d++)
    {
      for (long k=0; k<image.getHeight(); k++)
      {
        for (long i=0; i<image.getWidth(); i++)
        {
          T v=0;
          
          v=image.get(i, k, d);
          if (image.isValidS(v))
            image.setLimited(i, k, d, v-s);
        }
      }
    }
    
    return image;
}

template<class T> Image<T>& operator*=(Image<T> &image, double s)
{
    for (int d=0; d<image.getDepth(); d++)
    {
      for (long k=0; k<image.getHeight(); k++)
      {
        for (long i=0; i<image.getWidth(); i++)
        {
          T v=0;
          
          v=image.get(i, k, d);
          if (image.isValidS(v))
            image.setLimited(i, k, d, static_cast<typename Image<T>::work_t>(v*s));
        }
      }
    }
    
    return image;
}

template<class T> Image<T>& operator/=(Image<T> &image, double s)
{
    for (int d=0; d<image.getDepth(); d++)
    {
      for (long k=0; k<image.getHeight(); k++)
      {
        for (long i=0; i<image.getWidth(); i++)
        {
          T v=0;
          
          v=image.get(i, k, d);
          if (image.isValidS(v))
            image.setLimited(i, k, d, static_cast<typename Image<T>::work_t>(v/s));
        }
      }
    }
    
    return image;
}

template<class T> Image<T> operator+(const Image<T> &image, typename Image<T>::work_t s)
{
    Image<T> ret(image);
    
    ret+=s;
    
    return ret;
}

template<class T> Image<T> operator-(const Image<T> &image, typename Image<T>::work_t s)
{
    Image<T> ret(image);
    
    ret-=s;
    
    return ret;
}

template<class T> Image<T> operator*(const Image<T> &image, double s)
{
    Image<T> ret(image);
    
    ret*=s;
    
    return ret;
}

template<class T> Image<T> operator/(const Image<T> &image, double s)
{
    Image<T> ret(image);
    
    ret/=s;
    
    return ret;
}

template<class T> void remapImage(Image<T> &image, const Image<T> &map)
{
    assert(map.getWidth() >= image.absMaxValue()+1);
    assert(map.getDepth() == image.getDepth());
    
    for (long k=0; k<image.getHeight(); k++)
    {
      for (long i=0; i<image.getWidth(); i++)
      {
        if (image.isValid(i, k))
        {
          for (int d=0; d<image.getDepth(); d++)
            image.set(i, k, d, map.get(static_cast<int>(image.get(i, k, d)), 0, d));
        }
      }
    }
}

template<class T> void fillGammaMap(Image<T> &map, int depth, int from, int to, double g)
{
    for (int i=0; i<from && i<map.getWidth(); i++)
      map.setLimited(i, 0, depth, map.absMinValue());
    
    for (int i=0; i<to && i<map.getWidth(); i++)
      map.setLimited(i, 0, depth, map.absMaxValue()*pow(static_cast<double>(i-from)/(to-from), 1.0/g));
    
    for (int i=to; i<map.getWidth(); i++)
      map.setLimited(i, 0, depth, map.absMaxValue());
}

template<class T> void fillGammaMap(Image<T> &map, const Image<T> &image, double g)
{
    map.setSize(image.absMaxValue()+1, 1, image.getDepth());
    
    for (int d=0; d<image.getDepth(); d++)
      fillGammaMap(map, d, 0, image.absMaxValue(), g);
}

}

#endif
