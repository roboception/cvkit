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

long Polygon::getCenterX() const
{
  long ret=0;

  if (vertex.size() > 0)
  {
    for (size_t i=0; i<vertex.size(); i++)
    {
      ret+=vertex[i][0];
    }

    ret/=static_cast<long>(vertex.size());
  }

  return ret;
}

long Polygon::getCenterY() const
{
  long ret=0;

  if (vertex.size() > 0)
  {
    for (size_t i=0; i<vertex.size(); i++)
    {
      ret+=vertex[i][1];
    }

    ret/=static_cast<long>(vertex.size());
  }

  return ret;
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

void Polygon::reduce(float tolerance)
{
  size_t i=0;
  size_t k=1;

  while (i+1 < vertex.size())
  {
    // test if linear piece can be extended by one

    bool remove=true;
    if (k+1 < vertex.size())
    {
      // get line between vertex i and k+1 in Hesse normal form

      gmath::Vector2d N;
      N[0]=vertex[k+1][1]-vertex[i][1];
      N[1]=vertex[i][0]-vertex[k+1][0];
      N/=norm(N);

      double d=N[0]*vertex[i][0]+N[1]*vertex[i][1];

      // check if distance of vertices in between i and k+1 exceed tolerance

      remove=false;
      for (size_t j=i+1; j<=k; j++)
      {
        if (std::abs(N[0]*vertex[j][0]+N[1]*vertex[j][1]-d) > tolerance)
        {
          remove=true;
          break;
        }
      }
    }

    // remove all vertices between i and k

    if (remove)
    {
      vertex.erase(vertex.begin()+i+1, vertex.begin()+k);

      i++;
      k=i+1;
    }
    else
    {
      k++;
    }
  }
}

bool Polygon::isClosed(float tolerance) const
{
  bool ret=false;

  if (vertex.size() >= 3)
  {
    long dx=vertex[vertex.size()-1][0]-vertex[0][0];
    long dy=vertex[vertex.size()-1][1]-vertex[0][1];

    ret=(dx*dx+dy*dy <= tolerance*tolerance);
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

void Polygon::scaleCenter(double s)
{
  if (vertex.size() > 0)
  {
    double cx=0;
    double cy=0;

    for (size_t i=0; i<vertex.size(); i++)
    {
      cx+=vertex[i][0];
      cy+=vertex[i][1];
    }

    cx/=vertex.size();
    cy/=vertex.size();

    for (size_t i=0; i<vertex.size(); i++)
    {
      vertex[i][0]=static_cast<long>(s*(vertex[i][0]-cx)+cx);
      vertex[i][1]=static_cast<long>(s*(vertex[i][1]-cy)+cy);
    }
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

namespace
{

/*
  Advance coordinates to the next pixel with color c. Return false if the next
  pixel would be the start pixel or if there is no next pixel.
*/

inline bool nextContourPixel(long &x, long &y, int &from_dir, ImageU8 &mask, const ImageU8 &image,
  long sx, long sy)
{
  static const long dx[]={-1, -1,  0,  1, 1,  1,  0, -1};
  static const long dy[]={ 0,  1,  1,  1, 0, -1, -1, -1};
  gutil::uint8 c=image.get(x, y);
  int ret=false;

  mask.set(x, y, 0, 255); // mark current pixel as used

  int dir=(from_dir+1)%8;
  while (dir != from_dir && !ret)
  {
    long xx=x+dx[dir];
    long yy=y+dy[dir];

    if (xx == sx && yy == sy)
    {
      break;
    }

    if (xx >= 0 && yy >= 0 && xx < image.getWidth() && yy < image.getHeight())
    {
      if (image.get(xx, yy) == c && mask.get(xx, yy) == 0)
      {
        // advance to next pixel

        x=xx;
        y=yy;
        from_dir=(dir+4)%8;

        ret=true;
      }
    }

    dir=(dir+1)%8;
  }

  return ret;
}

}

void extractContour(Polygon &p, ImageU8 &mask, const ImageU8 &image, long sx, long sy, int dir)
{
  long x=sx;
  long y=sy;

  p.clear();
  p.add(x, y);

  while (nextContourPixel(x, y, dir, mask, image, sx, sy))
  {
    p.add(x, y);
  }
}

void extractLines(std::vector<Polygon> &pl, const ImageU8 &image, int c, int min_size, bool closed,
  float tolerance)
{
  ImageU8 mask(image.getWidth(), image.getHeight(), 1);
  mask.clear();

  for (long y=0; y<image.getHeight(); y++)
  {
    for (long x=0; x<image.getWidth(); x++)
    {
      Polygon p;

      if (image.get(x, y) == c && mask.get(x, y) == 0)
      {
        extractContour(p, mask, image, x, y, 0);

        if (p.count() >= min_size)
        {
          if (!closed || p.isClosed())
          {
            if (tolerance >= 0)
            {
              p.reduce(tolerance);
            }

            pl.push_back(p);
          }
        }
      }
    }
  }
}

void extractContours(std::vector<Polygon> &pl, const ImageU8 &image, float tolerance)
{
  uint8_t fg=0;

  for (long k=0; k<image.getHeight(); k++)
  {
    for (long i=0; i<image.getWidth(); i++)
    {
      fg=std::max(fg, image.get(i, k));
    }
  }

  ImageU8 mask(image.getWidth(), image.getHeight(), 1);
  mask.clear();

  for (long y=0; y<image.getHeight(); y++)
  {
    bool inside=false;
    for (long x=0; x<image.getWidth(); x++)
    {
      Polygon p;

      if (image.get(x, y) == fg && !inside)
      {
        if (mask.get(x, y) == 0)
        {
          extractContour(p, mask, image, x, y, 0);

          if (p.isClosed())
          {
            if (tolerance >= 0)
            {
              p.reduce(tolerance);
            }

            pl.push_back(p);
          }
        }

        inside=true;
      }
      else if (image.get(x, y) != fg && inside)
      {
        if (mask.get(x-1, y) == 0)
        {
          extractContour(p, mask, image, x-1, y, 4);

          if (p.isClosed())
          {
            if (tolerance >= 0)
            {
              p.reduce(tolerance);
            }

            pl.push_back(p);
          }
        }

        inside=false;
      }
    }
  }
}

}
