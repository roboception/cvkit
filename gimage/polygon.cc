/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2017 Roboception GmbH
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

#include "polygon.h"

#include <algorithm>

namespace gimage
{

VEdge::VEdge(long x0, long y0, long x1, long y1)
{
  if (y0 <= y1)
  {
    ymin=y0;
    ymax=y1;
    xstart=x0;
    xend=x1;
  }
  else if (y1 < y0)
  {
    ymin=y1;
    ymax=y0;
    xstart=x1;
    xend=x0;
  }

  xc=xstart;
}

void VEdge::setYCurrent(long yc)
{
  xc=(xend-xstart)*(yc-ymin)/(ymax-ymin)+xstart;
}

bool Polygon::isInside(long x, long y) const
{
  bool ret=false;

  if (vertex.size() >= 3)
  {
    size_t j=vertex.size()-1;
    for (size_t i=0; i<vertex.size(); i++)
    {
      if ((vertex[i][1] <= y && y < vertex[j][1]) || (vertex[j][1] <= y && y < vertex[i][1]))
      {
        if (x < (vertex[j][0]-vertex[i][0])*(y-vertex[i][1])/(vertex[j][1]-vertex[i][1])
                 +vertex[i][0])
        {
          ret=!ret;
        }
      }

      j=i;
    }
  }

  return ret;
}

void Polygon::addOffset(long x, long y)
{
  for (size_t i=0; i<vertex.size(); i++)
  {
    vertex[i][0]+=x;
    vertex[i][1]+=y;
  }
}

void Polygon::scale(double s)
{
  for (size_t i=0; i<vertex.size(); i++)
  {
    vertex[i][0]=static_cast<long>(s*vertex[i][0]);
    vertex[i][1]=static_cast<long>(s*vertex[i][1]);
  }
}

void Polygon::getVEdges(std::vector<VEdge> &list) const
{
  list.clear();

  size_t j=vertex.size()-1;
  for (size_t i=0; i<vertex.size(); i++)
  {
    VEdge v(vertex[i][0], vertex[i][1], vertex[j][0], vertex[j][1]);

    if (v.isValid())
    {
      list.push_back(v);
    }

    j=i;
  }

  std::sort(list.begin(), list.end(), compareY);
}

}