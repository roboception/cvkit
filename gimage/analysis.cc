/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2014, Institute of Robotics and Mechatronics, German Aerospace Center,
 *               2024, Roboception GmbH
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
#include "gauss.h"
#include "arithmetic.h"

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
      {
        row[k]=row[k-1]+w;
      }
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
    {
      colhist(i, 1)+=row[k][i];
    }
  }
}

void Histogram::sumCols(Histogram &rowhist) const
{
  rowhist.setSize(h, 1, bs);
  rowhist.clear();

  for (int k=0; k<h; k++)
  {
    for (int i=0; i<w; i++)
    {
      rowhist(k, 1)+=row[k][i];
    }
  }
}

unsigned long Histogram::sumAll() const
{
  unsigned long ret=0;

  for (int k=0; k<h; k++)
  {
    for (int i=0; i<w; i++)
    {
      ret+=row[k][i];
    }
  }

  return ret;
}

void Histogram::visualize(ImageU8 &image) const
{
  unsigned long maxval=1;

  for (int k=0; k<h; k++)
  {
    for (int i=0; i<w; i++)
    {
      maxval=std::max(maxval, row[k][i]);
    }
  }

  visualize(image, maxval);
}

void Histogram::visualize(ImageU8 &image, unsigned long maxval) const
{
  if (h > 1)
  {
    image.setSize(w, h, 1);

    for (int k=0; k<h; k++)
    {
      for (int i=0; i<w; i++)
      {
        image.set(i, k, 0, static_cast<unsigned char>(255-255*row[k][i]/maxval));
      }
    }
  }
  else
  {
    image.setSize(w, 256, 1);
    image.clear();

    for (int i=0; i<w; i++)
    {
      int k=255-255*val[i]/maxval;

      while (k >= 0)
      {
        image.set(i, k--, 0, 255);
      }
    }
  }
}

void Histogram::convertToImage(ImageFloat &image) const
{
  float total=static_cast<float>(sumAll());

  image.setSize(w, h, 1);

  if (total > 0)
  {
    for (int k=0; k<h; k++)
    {
      for (int i=0; i<w; i++)
      {
        image.set(i, k, 0, row[k][i]/total);
      }
    }
  }
}

namespace
{

inline float negLog(float &p)
{
  double v=-std::log(p);
  float ret=p*v;

  p=v;

  return ret;
}

}

float getMutualInformationU8(ImageFloat &mi, const ImageU8 &image1, const ImageU8 &image2,
  const ImageFloat &disp, int channel)
{
  ImageFloat P(256, 256, 1);
  P=0;

  // For computation of mutual information, see Hirschmueller (2008, TPAMI) or
  // Hirschmueller (2005, CVPR).

  // compute joint probabilities P_I1,I2

  float scale; // 1/number_of_correspondences

  if (disp.getWidth() > 0 && disp.getHeight() > 0)
  {
    long width=std::min(std::min(image1.getWidth(), image2.getWidth()), disp.getWidth());
    long height=std::min(std::min(image1.getHeight(), image2.getHeight()), disp.getHeight());
    long n=0;

    for (long k=0; k<height; k++)
    {
      for (long i=0; i<width; i++)
      {
        float v=disp(i, k, 0);

        if (disp.isValidS(v) && v < i)
        {
          v=i-v; // pixel in right image
          long ii=static_cast<long>(v); // full pixel
          v-=ii; // fraction

          // get intensity frpm right image by linear interpolation
          v=0.5f*((1-v)*image2(i, k, channel)+v*image2(i, k, channel));

          // increment corresponding cell in joint histogram
          P(image1(i, k, channel), static_cast<int>(v+0.5f))++;

          n++;
        }
      }
    }

    scale=1.0f/n;
    P*=scale;
  }
  else
  {
    long width=std::min(image1.getWidth(), image2.getWidth());
    long height=std::min(image1.getHeight(), image2.getHeight());

    for (int d=0; d<P.getDepth(); d++)
    {
      for (long k=0; k<height; k++)
      {
        for (long i=0; i<width; i++)
        {
          // increment corresponding cell in joint histogram
          P(image1(i, k, channel), image2(i, k, channel))++;
        }
      }
    }

    scale=1.0f/(width*height);
    P*=scale;
  }

  // replace all cells with 0 by the smallest value to avoid computing log(0)

  for (long k=0; k<P.getHeight(); k++)
  {
    for (long i=0; i<P.getWidth(); i++)
    {
      if (P(i, k) == 0) P(i, k)=scale;
    }
  }

  // compute joint entropy h_I1,I2 = -Gauss_convolution(log(Gauss_convolution(P_I1,I2)))

  ImageFloat Pg;
  gauss(Pg, P, 1.0f);

  double entropy=0;
  float *pg=Pg.getPtr(0, 0);

  for (long k=0; k<Pg.getHeight(); k++)
  {
    for (long i=0; i<Pg.getWidth(); i++)
    {
      entropy+=negLog(*pg);
      pg++;
    }
  }

  gauss(mi, Pg, 1.0f);

  // sum over columns for getting probabilities of image1

  ImageFloat P1(P.getWidth(), 1, 1);

  for (long i=0; i<P.getWidth(); i++)
  {
    float sum=0;
    for (long k=0; k<P.getHeight(); k++) sum+=P(i, k);
    P1(i, 0)=sum;
  }

  // compute entropy for image1

  GaussKernel gk(1.0f);

  ImageFloat Pg1(P.getWidth(), 1, 1);
  for (long i=0; i<P1.getWidth(); i++) Pg1(i, 0)=gk.convolveHorizontal(P1, i, 0, 0);

  double entropy1=0;
  pg=Pg1.getPtr(0, 0);

  for (long i=0; i<Pg1.getWidth(); i++)
  {
    entropy1+=negLog(*pg);
    pg++;
  }

  // convolve with Gauss again add to negative mi values

  for (long i=0; i<Pg1.getWidth(); i++)
  {
    float v=gk.convolveHorizontal(Pg1, i, 0, 0);
    for (long k=0; k<P.getHeight(); k++) mi(i, k)=v-mi(i, k);
  }

  // sum over rows for getting probabilities of image1

  P1.setSize(P.getHeight(), 1, 1);

  for (long k=0; k<P.getHeight(); k++)
  {
    float sum=0;
    for (long i=0; i<P.getWidth(); i++) sum+=P(i, k);
    P1(k, 0)=sum;
  }

  // compute entropy for image2

  Pg1.setSize(P.getHeight(), 1, 1);
  for (long i=0; i<Pg1.getWidth(); i++) Pg1(i, 0)=gk.convolveHorizontal(P1, i, 0, 0);

  double entropy2=0;
  pg=Pg1.getPtr(0, 0);

  for (long i=0; i<Pg1.getWidth(); i++)
  {
    entropy2+=negLog(*pg);
    pg++;
  }

  // convolve with Gauss again add to mi values

  for (long k=0; k<Pg1.getWidth(); k++)
  {
    float v=gk.convolveHorizontal(Pg1, k, 0, 0);
    for (long i=0; i<P.getWidth(); i++) mi(i, k)+=v;
  }

  // apply 1/n

  mi*=scale;

  return entropy1+entropy2-entropy;
}

}
