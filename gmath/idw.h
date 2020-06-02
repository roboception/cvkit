/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2020, Roboception GmbH
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

#ifndef GMATH_IDW_H
#define GMATH_IDW_H

#include <list>

namespace gmath
{

/**
 * Internally used node of the IDW quad-tree.
 */

class IDWNode
{
  public:

    /**
      Creates a leaf node with the given data.
    */

    IDWNode(unsigned int x, unsigned int y, float f, float w);

    /**
      Creates a parent node with the given child.

      NOTE: This constructor throws an exception if x or y is not a multple of
      s, or if the child is not inside the parent.

      @param x     left edge of node. The coordinate must be multiple of _s.
      @param y     top edge of the node. The coordinate must be a multiple of _s.
      @param s     Size of node.
      @param child Child to be added to this node. Its x, y location must be
                   inside the parent.
    */

    IDWNode(unsigned int x, unsigned int y, unsigned int s, IDWNode *child);

    ~IDWNode();

    /**
      Adds the given data point to the internal quad-tree structure.
      The returned is a pointer to this node or to a newly allocated
      node that replaces this node in the tree hierarchy.
    */

    IDWNode *add(unsigned int x, unsigned int y, float f, float w);

    /**
      Returns the number of data values below this node.
    */

    int getCount() { return n; }

    /**
      Get the interpolated value at x, y as fraction. The value is calculated
      by f = nominator / denominator.
    */

    void get(unsigned int x, unsigned int y, float &nominator, float &denominator);

    /**
      Gets the value of this node, which is the average of the values of all
      nodes below.
    */

    float getNodeValue() { return f; }

  private:

    IDWNode *allocParent(int x, int y, int s);

    IDWNode(class IDWNode &); // forbidden
    IDWNode &operator=(const IDWNode &); // forbidden

    unsigned int x, y, s;
    float r2, xs, ys;

    int n;
    float f, w;

    IDWNode *child[4];
};

/**
 * Efficient, approximate inverse distance weighted interpolation of 2D raster
 * data. The implementation internally uses a quad-tree structure.
 */

class IDW
{
  public:

    IDW(int id=-1);
    ~IDW();

    /**
      Sets an arbitrary ID. The ID is not used by the class itself, it is just
      returned upon request.
    */

    void setID(int id);
    int getID();

    /**
      Add a data value. fx and fy are the derivation of f in x and y direction
      and w is a weighting factor.

      @param x, y Location of data.
      @param f    Value at x, y.
      @param w    Weight of value.
    */

    void add(unsigned int x, unsigned int y, float f, float w=1);

    /**
      Returns the total number of data values.
    */

    int getCount();

    /**
      Returns the interpolated value at a given location.
    */

    float get(unsigned int x, unsigned int y);

    /**
      Returns the mean of all values.
    */

    float getMean();

  private:

    IDW(class IDW &); // forbidden
    IDW &operator=(const IDW &); // forbidden

    int id;
    IDWNode *root;
};

}

#endif
