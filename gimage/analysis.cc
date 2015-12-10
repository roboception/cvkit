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

#include "analysis.h"

using std::max;

namespace gimage
{

void Histogram::setSize(int width, int height, int binsize)
{
    if (w != width || h != height)
    {
      delete [] val;
      delete [] row;
      
      w=width;
      h=height;
      val=0;
      row=0;
      
      if (w > 0 && h > 0)
      {
        val=new unsigned long [w*h];
        row=new unsigned long * [h];
        
        row[0]=val;
        for (int k=1; k<h; k++)
          row[k]=row[k-1]+w;
      }
    }
    
    bs=binsize;
}
    
void Histogram::sumRows(Histogram &colhist) const
{
    colhist.setSize(w, 1, bs);
    colhist.clear();
    
    for (int k=0; k<h; k++)
    {
      for (int i=0; i<w; i++)
        colhist(i, 1)+=row[k][i];
    }
}

void Histogram::sumCols(Histogram &rowhist) const
{
    rowhist.setSize(h, 1, bs);
    rowhist.clear();
    
    for (int k=0; k<h; k++)
    {
      for (int i=0; i<w; i++)
        rowhist(k, 1)+=row[k][i];
    }
}

unsigned long Histogram::sumAll() const
{
    unsigned long ret=0;
    
    for (int k=0; k<h; k++)
    {
      for (int i=0; i<w; i++)
        ret+=row[k][i];
    }
    
    return ret;
}

void Histogram::visualize(ImageU8 &image) const
{
    unsigned long maxval=1;
    
    for (int k=0; k<h; k++)
      for (int i=0; i<w; i++)
        maxval=max(maxval, row[k][i]);
    
    if (h > 1)
    {
      image.setSize(w, h, 1);
      
      for (int k=0; k<h; k++)
        for (int i=0; i<w; i++)
          image.set(i, k, 0, static_cast<unsigned char>(255-255*row[k][i]/maxval));
    }
    else
    {
      image.setSize(w, 256, 1);
      image.clear();
      
      for (int i=0; i<w; i++)
      {
        int k=255-255*val[i]/maxval;
        
        while (k >= 0)
          image.set(i, k--, 0, 255);
      }
    }
}

void Histogram::convertToImage(ImageFloat &image) const
{
    float total=sumAll();
    
    image.setSize(w, h, 1);
    
    if (total > 0)
      for (int k=0; k<h; k++)
        for (int i=0; i<w; i++)
          image.set(i, k, 0, row[k][i]/total);
}

}
