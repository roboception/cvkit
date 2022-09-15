/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2022 Roboception GmbH
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

#include "hdr.h"

namespace gimage
{

HighDynamicRangeFusionBase::HighDynamicRangeFusionBase()
{
  // create lookup table for Gauss curve

  int n=256;
  gauss.resize(n);

  double inv_var=1.0/(51*51); // standard deviation of 0.2 for value range 0 to 1

  for (int i=0; i<n; i++)
  {
    int x=i-(n>>1);
    gauss[i]=static_cast<float>(std::exp(-0.5*x*x*inv_var));
  }
}

void HighDynamicRangeFusionBase::setContrastWeight(ImageFloat &wp, const ImageFloat &lp,
  float max_value)
{
  float scale=1.0f/max_value/lp.getDepth();

  float *p=wp.getPtr(0, 0, 0);
  for (long k=0; k<lp.getHeight(); k++)
  {
    for (long i=0; i<lp.getWidth(); i++)
    {
      float v=0;
      for (int d=0; d<lp.getDepth(); d++)
      {
        v+=lp.get(i, k, d);
      }

      // use absolute value of laplacian as measure for contrast

      if (v < 0) v=-v;

      *p++ = scale*v;
    }
  }
}

void HighDynamicRangeFusionBase::normalizeWeights(float scale)
{
  if (list.size() == 0)
  {
    return;
  }

  const int w=static_cast<int>(list[0]->weight[0].getWidth());
  const int h=static_cast<int>(list[0]->weight[0].getHeight());

  std::vector<float *> wp(list.size());
  for (size_t j=0; j<list.size(); j++)
  {
    wp[j]=list[j]->weight[0].getPtr(0, 0, 0);
  }

  for (int k=0; k<h; k++)
  {
    for (int i=0; i<w; i++)
    {
      float wsum=1e-12f; // avoid division by zero

      for (size_t j=0; j<wp.size(); j++)
      {
        wsum+=*wp[j];
      }

      float s=scale/wsum;

      for (size_t j=0; j<wp.size(); j++)
      {
        *wp[j]++ *= s;
      }
    }
  }
}

void HighDynamicRangeFusionBase::mulWeight(ImageFloat &target, ImageFloat &weight)
{
  for (int d=0; d<target.getDepth(); d++)
  {
    float *p=target.getPtr(0, 0, d);
    float *pn=p+target.getWidth()*target.getHeight();
    float *w=weight.getPtr(0, 0, 0);

    while (p < pn)
    {
      *p++ *= *w++;
    }
  }
}

void HighDynamicRangeFusionBase::addMulWeight(ImageFloat &target, ImageFloat &image, ImageFloat &weight)
{
  for (int d=0; d<target.getDepth(); d++)
  {
    float *p=target.getPtr(0, 0, d);
    float *pn=p+target.getWidth()*target.getHeight();
    float *im=image.getPtr(0, 0, d);
    float *w=weight.getPtr(0, 0, 0);

    while (p < pn)
    {
      *p++ += *im++ * *w++;
    }
  }
}

}
