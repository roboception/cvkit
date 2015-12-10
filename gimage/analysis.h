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

#ifndef GIMAGE_ANALYSIS_H
#define GIMAGE_ANALYSIS_H

#include "image.h"

namespace gimage
{

class Histogram
{
  private:
  
    int           w, h, bs;
    unsigned long *val;
    unsigned long **row;
    
  public:
    
    explicit Histogram(int width=0, int height=1, int binsize=1)
    {
      w=h=bs=0;
      val=0;
      row=0;
      
      setSize(width, height, binsize);
      clear();
    }
    
    Histogram(const Histogram &hist)
    {
      w=h=bs=0;
      val=0;
      row=0;
      
      setSize(hist.getWidth(), hist.getHeight(), hist.getBinSize());
      memcpy(val, hist.val, w*h*sizeof(unsigned long));
    }
    
      // computes a histogram of intensities from the given image
    
    template<class T> Histogram(const Image<T> &image, int binsize=1)
    {
      w=h=bs=0;
      val=0;
      row=0;
      
      int width=std::max(256, std::min(65536, static_cast<int>(image.maxValue())))/binsize;
      setSize(width, 1, binsize);
      clear();
      
      for (int k=0; k<image.getHeight(); k++)
      {
        for (int i=0; i<image.getWidth(); i++)
        {
          if (image.isValid(i, k))
          {
            for (int d=0; d<image.getDepth(); d++)
            {
              T v=image.get(i, k, d);
              
              if (v < w*binsize+binsize-1)
                val[static_cast<int>(v)/binsize]++;
            }
          }
        }
      }
    }
    
      // computes a correspondence histogram of intensities from the given images
    
    template<class T> Histogram(const Image<T> &image1, const Image<T> &image2, int binsize=1)
    {
      w=h=bs=0;
      val=0;
      row=0;
      
      int width=std::max(256, std::min(65536, static_cast<int>(image1.maxValue())))/binsize;
      int height=std::max(256, std::min(65536, static_cast<int>(image2.maxValue())))/binsize;
      setSize(width, height, binsize);
      clear();
      
      for (int k=0; k<image1.getHeight() && k < image2.getHeight(); k++)
      {
        for (int i=0; i<image1.getWidth() && i < image2.getWidth(); i++)
        {
          if (image1.isValid(i, k) && image2.isValid(i, k))
          {
            for (int d=0; d<image1.getDepth() && d < image2.getDepth(); d++)
            {
              T v1=image1.get(i, k, d);
              T v2=image2.get(i, k, d);
              
              if (v1 < w*binsize+binsize-1 && v2 < h*binsize+binsize-1)
                row[static_cast<int>(v2)/binsize][static_cast<int>(v1)/binsize]++;
            }
          }
        }
      }
    }
    
    ~Histogram()
    {
      delete [] val;
      delete [] row;
    }
    
    Histogram& operator=(const Histogram &hist)
    {
      setSize(hist.getWidth(), hist.getHeight(), hist.getBinSize());
      memcpy(val, hist.val, w*h*sizeof(unsigned long));
      
      return *this;
    }

    void setSize(int width, int height, int binsize);
    
    void clear()
    {
      if (w*h > 0)
        memset(val, 0, w*h*sizeof(unsigned long));
    }
    
    int getWidth() const
    {
      return w;
    }
    
    int getHeight() const
    {
      return h;
    }
    
    int getBinSize() const
    {
      return bs;
    }
    
    unsigned long& operator()(int i, int k=0)
    {
      return row[k][i];
    }
    
    unsigned long operator()(int i, int k=0) const
    {
      return row[k][i];
    }
    
      // sums all rows or columns of a 2D histogram and stores the resulting 1D histogram
    
    void sumRows(Histogram &colhist) const;
    void sumCols(Histogram &rowhist) const;
    unsigned long sumAll() const;
    
      // stores a visualization of the 1D or 2D histogram into the given image
    
    void visualize(ImageU8 &image) const;
    
      // stores the normalized 1D or 2D histogram into the given float image
    
    void convertToImage(ImageFloat &image) const;
};

}

#endif
