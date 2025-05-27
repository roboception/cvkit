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

#ifndef GIMAGE_POLYGON_H
#define GIMAGE_POLYGON_H

#include "image.h"
#include "paint.h"

#include <gmath/svector.h>

#include <vector>

namespace gimage
{

/**
  Helper class for storing vertical edges of a polygon.
*/

class VEdge
{
  public:

    VEdge(long x0, long y0, long x1, long y1);

    bool isValid() const { return ymin < ymax; }

    long getYMin() const { return ymin; }
    long getYMax() const { return ymax; }

    void setYCurrent(long yc);
    long getXCurrent() const { return xc; }

  private:

    long ymin;
    long ymax;
    long xstart;
    long xend;

    long xc;
};

inline bool compareX(const VEdge &v0, const VEdge &v1)
{
  return v0.getXCurrent() < v1.getXCurrent();
}

inline bool compareY(const VEdge &v0, const VEdge &v1)
{
  return v0.getYMin() < v1.getYMin();
}

/**
  Class for managing polygons that can be painted into an image.
*/

class Polygon
{
  private:

    std::vector<gmath::SVector<long, 2> > vertex;

    template<class T> void drawHorizontal(Image<T> &image, long x0, long x1, long y, T v[]) const
    {
      x0=std::max(0l, x0);
      x1=std::min(x1, image.getWidth()-1);

      int cn=std::min(3, image.getDepth());
      for (int c=0; c<cn; c++)
      {
        for (long x=x0; x<=x1; x++)
        {
          image.set(x, y, c, v[c]);
        }
      }
    }

  public:

    Polygon() {}

    Polygon(std::initializer_list<long> xy_pairs) { init(xy_pairs); }

    /**
      Initialization from a constant list of xy values.
    */

    void init(std::initializer_list<long> xy_pairs)
    {
      const long *p=xy_pairs.begin();
      while (p != xy_pairs.end())
      {
        long x=*p++;
        if (p != xy_pairs.end()) vertex.push_back(gmath::SVector<long, 2>(x, *p++));
      }
    }

    /**
      Returns the number of vertices.

      @return Number of vertices.
    */

    int count() const { return static_cast<int>(vertex.size()); }

    /**
      Returns the X coordinate of a vertex.

      @param i Index of vertex.
      @return  X coordinate of vertex.
    */

    long getX(int i) const { return vertex[i][0]; }

    /**
      Returns the Y coordinate of a vertex.

      @param i Index of vertex.
      @return  Y coordinate of vertex.
    */

    long getY(int i) const { return vertex[i][1]; }

    /**
      Returns the X coordinate of the center of all vertices.

      @return X coordinate of center.
    */

    long getCenterX() const;

    /**
      Returns the Y coordinate of the center of all vertices.

      @return Y coordinate of center.
    */

    long getCenterY() const;

    /**
      Checks if the given point is inside the polygon.

      @param x X coordinate of point;
      @param y X coordinate of point;
      @return  True if the point is inside the polygon.
    */

    bool isInside(long x, long y) const;

    /**
      Removes all vertices of the polygon.
    */

    void clear() { vertex.clear(); }

    /**
      Adds a vertex with the given coordinates at the end of the list of
      vertices.

      @param x X coordinate of vertex.
      @param y Y coordinate of vertex.
    */

    void add(long x, long y) { vertex.push_back(gmath::SVector<long, 2>(x, y)); }

    /**
      Remove all pixels of the polygon that are located on a straight line.

      @param tolerance Maximum tolerance between line and pixel to be removed.
    */

    void reduce(float tolerance=0.25f);

    /**
      Returns true if the first and last pixel are at most the tolerance
      distance away from each other.

      @param tolerance Maximum distance of first and last pixel for a closed
                       contour.
      @return          True if contour is closed.
    */

    bool isClosed(float tolerance=1.5f) const;

    /**
      Add an offset to all vertices.

      @param x X offset.
      @param y Y offset.
    */

    void addOffset(long x, long y);

    /**
      Scale all vertices by the given factor.

      @param s Scale factor.
    */

    void scale(double s);

    /**
      Scale all vertices by the given factor in relation to center of polygon.

      @param s Scale factor.
    */

    void scaleCenter(double s);

    /**
      Computes the list of non-horizontal edges of the polygon. The edges are
      sorted in ascending order by ymin.

      @param list List that will be filled with all non-horizontal edges of the
                  polygon.
    */

    void getVEdges(std::vector<VEdge> &list) const;

    /**
      Draws the polygon into the given image.

      @param image   Image to which the polygon is drawn.
      @param r, g, b Color for drawing.
    */

    template<class T> void draw(Image<T> &image, float r, float g=-1, float b=-1) const
    {
      size_t j=vertex.size()-1;
      for (size_t i=0; i<vertex.size(); i++)
      {
        drawLine(image, vertex[i][0], vertex[i][1], vertex[j][0], vertex[j][1], r, g, b);
        j=i;
      }
    }

    /**
      Paints a filled polygon into the given image.

      @param image   Image to which the polygon is drawn.
      @param r, g, b Color for drawing.
    */

    template<class T> void fill(Image<T> &image, float r, float g=-1, float b=-1) const
    {
      if (vertex.size() < 3)
      {
        return;
      }

      T v[3];

      if (g < 0) g=r;
      if (b < 0) b=r;

      v[0]=static_cast<T>(r);
      v[1]=static_cast<T>(g);
      v[2]=static_cast<T>(b);

      // get sorted list of vertical edges

      std::vector<VEdge> edge, used;
      getVEdges(edge);

      // go line by line through y range of polygon

      for (long y=std::max(0l, edge[0].getYMin()); y<image.getHeight() &&
                                                   edge.size()+used.size() > 0; y++)
      {
        // add egdes if y is larger or equal ymin

        while (edge.size() > 0 && y >= edge[0].getYMin())
        {
          used.push_back(edge[0]);
          edge.erase(edge.begin());
        }

        // remove edges if y is larger than ymax

        {
          size_t i=0;
          while (i < used.size())
          {
            if (y >= used[i].getYMax())
            {
              used.erase(used.begin()+i);
            }
            else
            {
              i++;
            }
          }
		}

        if (used.size() > 0)
        {
          // sort used edges according to current X coordinate

          for (size_t i=0; i<used.size(); i++)
          {
            used[i].setYCurrent(y);
          }

          std::sort(used.begin(), used.end(), compareX);

          // draw horizontal lines between used edges

          for (size_t i=0; i<used.size()-1; i+=2)
          {
            drawHorizontal(image, used[i].getXCurrent(), used[i+1].getXCurrent(), y, v);
          }
        }
      }
    }

    /**
      Fills everything outside the polygon in the given image.

      @param image   Image to which the polygon is drawn.
      @param r, g, b Color for drawing.
    */

    template<class T> void fillOutside(Image<T> &image, float r, float g=-1, float b=-1) const
    {
      if (vertex.size() < 3)
      {
        return;
      }

      T v[3];

      if (g < 0) g=r;
      if (b < 0) b=r;

      v[0]=static_cast<T>(r);
      v[1]=static_cast<T>(g);
      v[2]=static_cast<T>(b);

      // get sorted list of vertical edges

      std::vector<VEdge> edge, used;
      getVEdges(edge);

      // fill everything above and below the polygon

      for (long y=0; y<edge[0].getYMin(); y++)
      {
        drawHorizontal(image, 0, image.getWidth()-1, y, v);
      }

      for (long y=edge[edge.size()-1].getYMax(); y<image.getHeight(); y++)
      {
        drawHorizontal(image, 0, image.getWidth()-1, y, v);
      }

      // go line by line through y range of polygon

      for (long y=std::max(0l, edge[0].getYMin()); y<image.getHeight() &&
                                                   edge.size()+used.size() > 0; y++)
      {
        // add egdes if y is larger or equal ymin

        while (edge.size() > 0 && y >= edge[0].getYMin())
        {
          used.push_back(edge[0]);
          edge.erase(edge.begin());
        }

        // remove edges if y is larger than ymax

        size_t i=0;
        while (i < used.size())
        {
          if (y >= used[i].getYMax())
          {
            used.erase(used.begin()+i);
          }
          else
          {
            i++;
          }
        }

        if (used.size() > 0)
        {
          // sort used edges according to current X coordinate

          for (size_t i=0; i<used.size(); i++)
          {
            used[i].setYCurrent(y);
          }

          std::sort(used.begin(), used.end(), compareX);

          // draw horizontal lines outside used edges

          drawHorizontal(image, 0, used[0].getXCurrent(), y, v);

          for (size_t i=1; i<used.size()-1; i+=2)
          {
            drawHorizontal(image, used[i].getXCurrent(), used[i+1].getXCurrent(), y, v);
          }

          drawHorizontal(image, used[used.size()-1].getXCurrent(), image.getWidth()-1, y, v);
        }
      }
    }
};

/**
  Extracts a polygon by starting at the given x, y coordinate and following
  the contour of pixels with the same intensity. If a polygon is approached
  from the outside, the order will be counter-clockwise. If it is approached
  from the inside, the order will be clockwise. All extracted pixels
  will be set to their bit-inverse value.

  @param p     Polygon to be set to extracted contour.
  @param mask  Mask image of same size of image. Corresponding pixel in image
               will only be considered if mask pixel is 0. Extracted pixels
               will be set to 255 in the mask image.
  @param image Input image. Only the first color channel will be considered.
  @param sx    x-coordinate of first pixel.
  @param sy    y-coordinate of first pixel.
  @param dir   Direction from which the first pixel is approached, between 0
               and 7 in the order 0 = from left, 1 = from bottom/left,
               2 = from bottom, etc.
*/

void extractContour(Polygon &p, ImageU8 &mask, const ImageU8 &image, long sx, long sy, int dir);

/**
  Extracts all lines of a minimum size as polygons from an image. The points of
  all polygons are ordered counter-clockwise.

  @param pl        List of polygons to be extracted.
  @param image     Input image. Only the first color channel will be considered.
  @param c         Color of lines.
  @param min_size  Minimum size of contours to be returned.
  @param closed    If true, the only closed contours are returned as polygons.
  @param tolerance Tolerance, used for reducing vertices of polygon.
*/

void extractLines(std::vector<Polygon> &pl, const ImageU8 &image, int c, int min_size=0,
  bool closed=true, float tolerance=0.25f);

/**
  Extracts contours of a binary image. The highest intensity is foreground, all
  lower intensities are background. Outer contours are ordered
  counter-clockwise, inner contours are ordered clockwise.

  @param pl        List of polygons to be extracted.
  @param image     Input image. Only the first color channel will be considered.
  @param tolerance Tolerance, used for reducing vertices of polygon.
*/

void extractContours(std::vector<Polygon> &pl, const ImageU8 &image, float tolerance=0.25f);

}

#endif
