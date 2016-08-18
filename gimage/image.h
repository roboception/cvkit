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

#ifndef GIMAGE_IMAGE_H
#define GIMAGE_IMAGE_H

#include <gutil/fixedint.h>

#include <limits>
#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <cstring>
#include <cmath>
#include <vector>

#include <iostream>

namespace gimage
{

/**
 * Definition of traits for common, scalar pixel types. The store type
 * should be as small as possible for not wasting memory. The work type
 * must be large enough for performing arithmetical operations without
 * overflow.
 */

template<class T> struct PixelTraits
{ };

template<>
struct PixelTraits<gutil::uint8>
{
    typedef gutil::uint8 store_t;
    typedef int          work_t;

    static inline const char *description() { return "uint8"; }
    static inline work_t minValue()         { return 0; }
    static inline work_t maxValue()         { return 255; }
    static inline store_t limit(work_t v)   { return static_cast<store_t>(std::max(0, std::min(255, v))); }
    static inline work_t invalid()          { return -1; }
    static inline bool isValidW(work_t v)   { return v >= 0 && v <= 255; }
    static inline bool isValidS(store_t v)  { return true; }
};

template<>
struct PixelTraits<gutil::uint16>
{
    typedef gutil::uint16 store_t;
    typedef int           work_t;

    static inline const char *description() { return "uint16"; }
    static inline work_t minValue()         { return 0; }
    static inline work_t maxValue()         { return 65535; }
    static inline store_t limit(work_t v)   { return static_cast<store_t>(std::max(0, std::min(65535, v))); }
    static inline work_t invalid()          { return -1; }
    static inline bool isValidW(work_t v)   { return v >= 0 && v <= 65535; }
    static inline bool isValidS(store_t v)  { return true; }
};

template<>
struct PixelTraits<gutil::uint32>
{
    typedef gutil::uint32 store_t;
    typedef gutil::int64  work_t;

    static inline const char *description() { return "uint32"; }
    static inline work_t minValue()         { return 0; }
    static inline work_t maxValue()         { return std::numeric_limits<store_t>::max(); }
    static inline store_t limit(work_t v)   { return static_cast<store_t>(std::max(minValue(), std::min(maxValue(), v))); }
    static inline work_t invalid()          { return -1; }
    static inline bool isValidW(work_t v)   { return v >= 0 && v <= static_cast<long long>(std::numeric_limits<store_t>::max()); }
    static inline bool isValidS(store_t v)  { return true; }
};

template<>
struct PixelTraits<float>
{
    typedef float store_t;
    typedef float work_t;

    static inline const char *description() { return "float"; }
    static inline work_t minValue()         { return -std::numeric_limits<float>::max(); }
    static inline work_t maxValue()         { return std::numeric_limits<float>::max(); }
    static inline store_t limit(work_t v)   { return v; }
    static inline work_t invalid()          { return std::numeric_limits<float>::infinity(); }
    static inline bool isValidW(work_t v)   { return std::isfinite(v); }
    static inline bool isValidS(store_t v)  { return std::isfinite(v); }
};

/**
 * Definition of an image.
 */

template<class T, class traits=PixelTraits<T> > class Image
{
  private:

    int  depth;
    long width, height, n;
    T    *pixel;
    T    **row;
    T    ***img;

  public:

    typedef traits                   ptraits;
    typedef T                        store_t; /**< store type for pixels */
    typedef typename ptraits::work_t work_t;  /**< work type for pixels */

    Image(long w=0, long h=0, int d=1)
    {
      depth=0;
      width=0;
      height=0;
      n=0;
      pixel=0;
      row=0;
      img=0;

      setSize(w, h, d);
    }

    Image(const Image<T> &a)
    {
      depth=0;
      width=0;
      height=0;
      n=0;
      pixel=0;
      row=0;
      img=0;

      setSize(a.getWidth(), a.getHeight(), a.getDepth());

      memcpy(pixel, a.pixel, n*sizeof(T));
    }

    ~Image()
    {
      delete [] pixel;
      delete [] row;
      delete [] img;
    }

    void setSize(long w, long h, long d)
    {
      if (width != w || height != h || depth != d)
      {
        if (pixel != 0)
          delete [] pixel;

        if (row != 0)
          delete [] row;

        if (img != 0)
          delete [] img;

        depth=d;
        width=w;
        height=h;
        n=width*height*depth;

        pixel=0;
        row=0;
        img=0;

        if (n > 0)
        {
          long m=height*depth;

          pixel=new T[n];
          row=new T*[m];
          img=new T**[depth];

          row[0]=pixel;
          for (long k=1; k<m; k++)
            row[k]=row[k-1]+width;

          img[0]=row;
          for (int j=1; j<depth; j++)
            img[j]=img[j-1]+height;
        }
      }
    }

    Image<T>& operator=(const Image<T> &a)
    {
      setSize(a.getWidth(), a.getHeight(), a.getDepth());

      memcpy(pixel, a.pixel, n*sizeof(T));

      return *this;
    }

    void clear()
    {
      store_t inv=ptraits::limit(ptraits::invalid());

      if (inv == 0)
      {
        memset(pixel, 0, n*sizeof(T));
      }
      else
      {
        for (int i=0; i<n; i++)
          pixel[i]=inv;
      }
    }

    long getWidth() const
    {
      return width;
    }

    long getHeight() const
    {
      return height;
    }

    int getDepth() const
    {
      return depth;
    }

    const char *getTypeDescription() const
    {
      return ptraits::description();
    }

    bool isValidS(store_t v) const
    {
      return ptraits::isValidS(v);
    }

    bool isValidW(work_t v) const
    {
      return ptraits::isValidW(v);
    }

    bool isValid(long i, long k) const
    {
      for (int j=0; j<depth; j++)
      {
        if (!isValidS(img[j][k][i]))
          return false;
      }

      return true;
    }

    /**
      Returns the pointer to the internal pixel buffer at the given position for
      fast, direct access.

      Increment for next pixel in same row: 1
      Increment for pixel in next row is: getWidth()
      Increment for same pixel in next color channel: getWidth()*getHeight()
    */

    T *getPtr(long i, long k, int j=0) const
    {
      return &(img[j][k][i]);
    }

    store_t get(long i, long k, int j=0) const
    {
      return img[j][k][i];
    }

    work_t getW(long i, long k, int j=0) const
    {
      return static_cast<work_t>(img[j][k][i]);
    }

    /**
     * Access with out-of-bound check (use nearest value within image
     * if out-of-bounds)
     */

    work_t getBounds(long i, long k, int j=0) const
    {
      i=std::max(0l, i);
      i=std::min(width-1, i);

      k=std::max(0l, k);
      k=std::min(height-1, k);

      j=std::max(0, j);
      j=std::min(depth-1, j);

      return static_cast<work_t>(img[j][k][i]);
    }

    void getBounds(std::vector<work_t> &p, long i, long k) const
    {
      i=std::max(0l, i);
      i=std::min(width-1, i);

      k=std::max(0l, k);
      k=std::min(height-1, k);

      for (int j=0; j<depth; j++)
        p[j]=static_cast<work_t>(img[j][k][i]);
    }

    /**
     * Access with out-of-bound check (return invalid, if
     * out-of-bounds)
     */

    work_t getBoundsInv(long i, long k, int j=0) const
    {
      work_t ret=ptraits::invalid();

      if (j >= 0 && j < depth && i >= 0 && i < width && k >= 0 && k < height)
        ret=static_cast<work_t>(img[j][k][i]);

      return ret;
    }

    void getBoundsInv(std::vector<work_t> &p, long i, long k) const
    {
      if (i >= 0 && i < width && k >= 0 && k < height)
      {
        for (int j=0; j<depth; j++)
          p[j]=static_cast<work_t>(img[j][k][i]);
      }
      else
      {
        for (int j=0; j<depth; j++)
          p[j]=ptraits::invalid();
      }
    }

    /**
     * Access with out-of-bound check (returns the nearest
     * valid neighbor, if out-of-bounds, and returns invalid,
     * if one if the four neighbors are invalid)
     */

    work_t getBilinear(float x, float y, int j=0) const
    {
      long    i, k;
      store_t p0, p1, p2, p3;
      work_t  ret=ptraits::invalid();

      j=std::max(0, j);
      j=std::min(depth-1, j);

      x-=0.5f;
      y-=0.5f;

      if (x < 0)
        x=0;

      if (y < 0)
        y=0;

      if (x >= width-1)
        x=width-1.001f;

      if (y >= height-1)
        y=height-1.001f;

      i=static_cast<long>(x);
      k=static_cast<long>(y);

      x-=i;
      y-=k;

      p0=img[j][k][i];
      p1=img[j][k][i+1];
      p2=img[j][k+1][i];
      p3=img[j][k+1][i+1];

      if (isValidS(p0) && isValidS(p1) && isValidS(p2) && isValidS(p3))
      {
        x*=4;
        y*=4;

        ret=p0*(4-x)*(4-y)+p1*x*(4-y)+p2*(4-x)*y+p3*x*y;
        ret/=16;
      }

      return ret;
    }

    void getBilinear(std::vector<work_t> &p, float x, float y) const
    {
      long    i, k;
      store_t p0, p1, p2, p3;

      x-=0.5f;
      y-=0.5f;

      if (x < 0)
        x=0;

      if (y < 0)
        y=0;

      if (x >= width-1)
        x=width-1.001f;

      if (y >= height-1)
        y=height-1.001f;

      i=static_cast<long>(x);
      k=static_cast<long>(y);

      x-=i;
      y-=k;

      x*=4;
      y*=4;

      const float s0=(4-x)*(4-y);
      const float s1=x*(4-y);
      const float s2=(4-x)*y;
      const float s3=x*y;

      for (int j=0; j<depth; j++)
      {
        p[j]=ptraits::invalid();

        p0=img[j][k][i];
        p1=img[j][k][i+1];
        p2=img[j][k+1][i];
        p3=img[j][k+1][i+1];

        if (isValidS(p0) && isValidS(p1) && isValidS(p2) && isValidS(p3))
          p[j]=(p0*s0+p1*s1+p2*s2+p3*s3)/16;
      }
    }

    /**
     * Access with out-of-bound check (returns the nearest
     * valid neighbor, if out-of-bounds, and returns invalid,
     * if one if the four neighbors are invalid)
     */

    work_t getBicubic(float x, float y, int j=0) const
    {
      if (x >= 1.5f && x+1.5f < width && y >= 1.5f && y+1.5f < height)
      {
        long i, k;

        j=std::max(0, j);
        j=std::min(depth-1, j);

        x-=0.5f;
        y-=0.5f;

        i=static_cast<long>(x);
        k=static_cast<long>(y);

        x-=i;
        y-=k;

        float v[4];
        for (int kk=0; kk<4; kk++)
        {
          const T *p=img[j][k-1+kk]+i-1;

          if (!isValidS(p[0]) || !isValidS(p[1]) || !isValidS(p[2]) || !isValidS(p[3]))
            return ptraits::invalid();

          const float p0=static_cast<float>(p[0]);
          const float p1=static_cast<float>(p[1]);
          const float p2=static_cast<float>(p[2]);
          const float p3=static_cast<float>(p[3]);

          v[kk]=p1+0.5f*x*(p2-p0+x*(2*p0-5*p1+4*p2-p3+x*(3*(p1-p2)+p3-p0)));
        }

        return ptraits::limit(v[1]+0.5f*y*(v[2]-v[0]+y*(2*v[0]-5*v[1]+4*v[2]-v[3]+y*(3*(v[1]-v[2])+
                                                                                     v[3]-v[0]))));
      }

      return getBilinear(x, y, j);
    }

    void getBicubic(std::vector<work_t> &p, float x, float y) const
    {
      if (x >= 1.5f && x+1.5f < width && y >= 1.5f && y+1.5f < height)
      {
        long i, k;

        x-=0.5f;
        y-=0.5f;

        i=static_cast<long>(x);
        k=static_cast<long>(y);

        x-=i;
        y-=k;

        float x2=x*x;
        float x3=x2*x;

        float a[4];
        a[0]=-0.5f*x+x2-0.5f*x3;
        a[1]=1-2.5f*x2+1.5f*x3;
        a[2]=0.5f*x+2*x2-1.5f*x3;
        a[3]=-0.5*x2+0.5*x3;

        float y2=y*y;
        float y3=y2*y;

        float b[4];
        b[0]=-0.5f*y+y2-0.5f*y3;
        b[1]=1-2.5f*y2+1.5f*y3;
        b[2]=0.5f*y+2*y2-1.5f*y3;
        b[3]=-0.5*y2+0.5*y3;

        for (int j=0; j<depth; j++)
        {
          float res=0;
          for (int kk=0; kk<4; kk++)
          {
            const T *v=img[j][k-1+kk]+i-1;

            if (!isValidS(v[0]) || !isValidS(v[1]) || !isValidS(v[2]) || !isValidS(v[3]))
            {
              res=ptraits::invalid();
              break;
            }

            res+=b[kk]*(a[0]*v[0]+a[1]*v[1]+a[2]*v[2]+a[3]*v[3]);
          }

          p[j]=ptraits::limit(res);
        }
      }
      else
      {
        getBilinear(p, x, y);
      }
    }

    work_t absMinValue() const
    {
      return ptraits::minValue();
    }

    work_t absMaxValue() const
    {
      return ptraits::maxValue();
    }

    work_t minValue() const
    {
      work_t ret=ptraits::maxValue();

      for (long i=0; i<n; i++)
      {
        if (isValidS(pixel[i]))
          ret=std::min(ret, static_cast<work_t>(pixel[i]));
      }

      return ret;
    }

    work_t maxValue() const
    {
      work_t ret=ptraits::minValue();

      for (long i=0; i<n; i++)
      {
        if (isValidS(pixel[i]))
          ret=std::max(ret, static_cast<work_t>(pixel[i]));
      }

      return ret;
    }

    void set(long i, long k, int j, store_t v)
    {
      img[j][k][i]=v;
    }

    void setLimited(long i, long k, int j, work_t v)
    {
      img[j][k][i]=ptraits::limit(v);
    }

    template<class S> void setImageLimited(const Image<S> &a)
    {
      setSize(a.getWidth(), a.getHeight(), a.getDepth());

      for (int j=0; j<depth; j++)
        for (long k=0; k<height; k++)
          for (long i=0; i<width; i++)
            img[j][k][i]=ptraits::limit(static_cast<work_t>(a.get(i, k, j)));
    }

    template<class S> void setImage(const Image<S> &a)
    {
      setSize(a.getWidth(), a.getHeight(), a.getDepth());

      for (int j=0; j<depth; j++)
        for (long k=0; k<height; k++)
          for (long i=0; i<width; i++)
            img[j][k][i]=static_cast<store_t>(a.get(i, k, j));
    }

    void setImage(const Image<T> &a)
    {
      setSize(a.getWidth(), a.getHeight(), a.getDepth());
      memcpy(pixel, a.getPtr(0, 0, 0), n*sizeof(T));
    }

    void setInvalid(long i, long k, long j)
    {
      img[j][k][i]=ptraits::limit(ptraits::invalid());
    }

    /**
     * Copy image content from and to an array. The array must have the size
     * getWidth()*getHeight()*getDepth(). Pixels are stored top down, line by
     * line from left to right. The values for all depth levels are stored
     * sequentielly.
     */

    void copyFrom(const store_t *p)
    {
      if (depth > 1)
      {
        for (long k=0; k<height; k++)
          for (long i=0; i<width; i++)
            for (int j=0; j<depth; j++)
              img[j][k][i]=*p++;
      }
      else
        memcpy(pixel, p, width*height*sizeof(store_t));
    }

    void copyTo(store_t *p) const
    {
      if (depth > 1)
      {
        for (long k=0; k<height; k++)
          for (long i=0; i<width; i++)
            for (int j=0; j<depth; j++)
              *p++=img[j][k][i];
      }
      else
        memcpy(p, pixel, width*height*sizeof(store_t));
    }
};

/**
 * Definition of the most common image types.
 */

typedef Image<gutil::uint8>  ImageU8;
typedef Image<gutil::uint16> ImageU16;
typedef Image<gutil::uint32> ImageU32;
typedef Image<float>         ImageFloat;

}

#endif