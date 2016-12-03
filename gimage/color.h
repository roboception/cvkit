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

#ifndef GIMAGE_COLOR_H
#define GIMAGE_COLOR_H

#include "image.h"

#include <gutil/exception.h>

#include <cmath>
#include <assert.h>

namespace gimage
{

/**
 * Converts between RGB and HSV pixels.
 *
 * 0 <= hue <= 6
 * 0 <= sat <= 1
 * 0 <= val <= 1
 */

template<class T>
void rgbToHSV(float &hue, float &sat, float &val, T red, T green, T blue, T maxval)
{
    float delta, vmin, vmax;

    vmin=red;
    vmin=std::min(vmin, static_cast<float>(green));
    vmin=std::min(vmin, static_cast<float>(blue));

    vmax=red;
    vmax=std::max(vmax, static_cast<float>(green));
    vmax=std::max(vmax, static_cast<float>(blue));

    val=vmax/maxval;
    delta=vmax-vmin;

    if (delta > 0)
    {
      if (red == vmax)
        hue=(green-blue)/delta;
      else if (green == vmax)
        hue=2+(blue-red)/delta;
      else
        hue=4+(red-green)/delta;

      if (hue < 0)
        hue+=6;

      sat=delta/vmax;
    }
    else
    {
      hue=0;
      sat=0;
    }
}

template<class T>
void hsvToRGB(T &red, T &green, T &blue, T maxval, float hue, float sat, float val)
{
    val*=maxval;

    if (sat != 0)
    {
      int   i=static_cast<int>(floorf(hue));
      float f=hue-i;
      float p=val*(1-sat);
      float q=val*(1-f*sat);
      float t=val*(1-sat*(1-f));

      switch (i)
      {
        case 0:
            red=static_cast<T>(val+0.5f);
            green=static_cast<T>(t+0.5f);
            blue=static_cast<T>(p+0.5f);
            break;

        case 1:
            red=static_cast<T>(q+0.5f);
            green=static_cast<T>(val+0.5f);
            blue=static_cast<T>(p+0.5f);
            break;

        case 2:
            red=static_cast<T>(p+0.5f);
            green=static_cast<T>(val+0.5f);
            blue=static_cast<T>(t+0.5f);
            break;

        case 3:
            red=static_cast<T>(p+0.5f);
            green=static_cast<T>(q+0.5f);
            blue=static_cast<T>(val+0.5f);
            break;

        case 4:
            red=static_cast<T>(t+0.5f);
            green=static_cast<T>(p+0.5f);
            blue=static_cast<T>(val+0.5f);
            break;

        default:
            red=static_cast<T>(val+0.5f);
            green=static_cast<T>(p+0.5f);
            blue=static_cast<T>(q+0.5f);
            break;
      }
    }
    else
      red=green=blue=static_cast<T>(val+0.5f);
}

/**
 * Interprets a three component image as RGB and converts it to a single
 * component grey image. Fails if image does not have three components.
 */

template<class T>
void imageToGrey(Image<T> &ret, const Image<T> &image)
{
    T red, green, blue;

    assert(image.getDepth() == 3);

    ret.setSize(image.getWidth(), image.getHeight(), 1);

    for (long k=0; k<image.getHeight(); k++)
    {
      for (long i=0; i<image.getWidth(); i++)
      {
        red=image.get(i, k, 0);
        green=image.get(i, k, 1);
        blue=image.get(i, k, 2);

        if (image.isValidS(red) && image.isValidS(green) && image.isValidS(blue))
        {
          ret.setLimited(i, k, 0,
            (9798*static_cast<typename Image<T>::work_t>(red)
            +19234*static_cast<typename Image<T>::work_t>(green)
            +3736*static_cast<typename Image<T>::work_t>(blue))/32768);
        }
        else
          ret.setInvalid(i, k, 0);
      }
    }
}

/**
 * Returns a color image from an intensity image by using the same value for R, G and B.
 */

template<class T, class traits>
void imageToColor(Image<T, traits> &ret, const Image<T, traits> &image)
{
    assert(image.getDepth() == 1);

    ret.setSize(image.getWidth(), image.getHeight(), 3);

    for (long k=0; k<image.getHeight(); k++)
    {
      for (long i=0; i<image.getWidth(); i++)
      {
        ret.set(i, k, 0, image.get(i, k, 0));
        ret.set(i, k, 1, image.get(i, k, 0));
        ret.set(i, k, 2, image.get(i, k, 0));
      }
    }
}

/**
 * Returns an 8 bit color image from an intensity image or from color channel 0
 * of a color image using JET color encoding.
 */

template<class T>
void imageToJET(ImageU8 &ret, const Image<T> &image, double imin=0, double imax=-1)
{
    ret.setSize(image.getWidth(), image.getHeight(), 3);

    if (imax <= imin)
    {
      imin=image.minValue();
      imax=image.maxValue();
    }

    double irange=imax-imin;

    if (irange <= 0)
    {
      ret.clear();
      return;
    }

    for (long k=0; k<image.getHeight(); k++)
    {
      for (long i=0; i<image.getWidth(); i++)
      {
        if (image.isValid(i, k))
        {
          double v=image.get(i, k, 0);

          v=(v-imin)/irange;
          v=v/1.15+0.1;

          double r=std::max(0.0, std::min(1.0, (1.5 - 4*fabs(v-0.75))));
          double g=std::max(0.0, std::min(1.0, (1.5 - 4*fabs(v-0.5))));
          double b=std::max(0.0, std::min(1.0, (1.5 - 4*fabs(v-0.25))));

          ret.set(i, k, 0, static_cast<unsigned char>(255*r+0.5));
          ret.set(i, k, 1, static_cast<unsigned char>(255*g+0.5));
          ret.set(i, k, 2, static_cast<unsigned char>(255*b+0.5));
        }
        else
        {
          ret.set(i, k, 0, 0);
          ret.set(i, k, 1, 0);
          ret.set(i, k, 2, 0);
        }
      }
    }
}

/**
 * Returns an image of one component of a multi-component image.
 */

template<class T, class traits>
void imageSelect(Image<T, traits> &ret, const Image<T, traits> &image, int j)
{
    ret.setSize(image.getWidth(), image.getHeight(), 1);

    for (long k=0; k<image.getHeight(); k++)
    {
      for (long i=0; i<image.getWidth(); i++)
        ret.set(i, k, 0, image.get(i, k, j));
    }
}

/**
 * Returns a multi-component image from three scalar images.
 */

template<class T, class traits>
void imageCompose(Image<T, traits> &ret, const Image<T, traits> &r,
  const Image<T, traits> &g, const Image<T, traits> &b)
{
    assert(r.getWidth() == g.getWidth());
    assert(r.getHeight() == g.getHeight());
    assert(r.getWidth() == b.getWidth());
    assert(r.getHeight() == b.getHeight());

    ret.setSize(r.getWidth(), r.getHeight(), 3);

    for (long k=0; k<r.getHeight(); k++)
    {
      for (long i=0; i<r.getWidth(); i++)
      {
        ret.set(i, k, 0, r(i, k));
        ret.set(i, k, 1, g(i, k));
        ret.set(i, k, 2, b(i, k));
      }
    }
}

/**
 * Converts between three component images, interpreted as RGB and HSV
 * Fails if the input image does not have three components.
 */

template<class T, class traits>
void rgbToHSV(ImageFloat &ret, const Image<T, traits> &image)
{
    float hue, sat, val;

    assert(image.getDepth() == 3);

    ret.setSize(image.getWidth(), image.getHeight(), 3);

    for (long k=0; k<image.getHeight(); k++)
    {
      for (long i=0; i<image.getWidth(); i++)
      {
        rgbToHSV(hue, sat, val, image.getW(i, k, 0), image.getW(i, k, 1),
          image.getW(i, k, 2), image.absMaxValue());

        ret.setLimited(i, k, 0, hue);
        ret.setLimited(i, k, 1, sat);
        ret.setLimited(i, k, 2, val);
      }
    }
}

template<class T, class traits>
void hsvToRGB(Image<T, traits> &ret, const ImageFloat &image)
{
    typename Image<T, traits>::work_t red, green, blue;

    assert(image.getDepth() == 3);

    ret.setSize(image.getWidth(), image.getHeight(), 3);

    for (long k=0; k<image.getHeight(); k++)
    {
      for (long i=0; i<image.getWidth(); i++)
      {
        hsvToRGB(red, green, blue, ret.absMaxValue(), image.getW(i, k, 0),
          image.getW(i, k, 1), image.getW(i, k, 2));

        ret.setLimited(i, k, 0, red);
        ret.setLimited(i, k, 1, green);
        ret.setLimited(i, k, 2, blue);
      }
    }
}

}

#endif