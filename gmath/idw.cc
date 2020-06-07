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

#include "idw.h"

#include <gutil/exception.h>

namespace gmath
{

IDWNode::IDWNode(unsigned int _x, unsigned int _y, float _f, float _w)
{
  x=_x;
  y=_y;
  s=1;

  r2=5*5;
  xs=x;
  ys=y;

  n=1;
  f=_f;
  w=_w;

  child[0]=0;
  child[1]=0;
  child[2]=0;
  child[3]=0;
}

IDWNode::IDWNode(unsigned int _x, unsigned int _y, unsigned int _s, IDWNode *_child)
{
  // sanity checks

  if (_x/_s*_s != _x || _y/_s*_s != _y)
  {
    throw gutil::InvalidArgumentException(std::string("IDWNode: Illegal location"));
  }

  if (_child->x < _x || _child->x >= _x+_s || _child->y < _y || _child->y >= _y+_s)
  {
    throw gutil::InvalidArgumentException(std::string("IDWNode: Child is not covered by parent"));
  }

  x=_x;
  y=_y;
  s=_s;

  r2=25*s*s;
  xs=x+0.5f*s-0.5f;
  ys=y+0.5f*s-0.5f;

  // add child

  child[0]=0;
  child[1]=0;
  child[2]=0;
  child[3]=0;

  int i;
  if (_child->x < x+(s>>1))
  {
    i=2;
    if (_child->y < y+(s>>1))
    {
      i=0;
    }
  }
  else
  {
    i=3;
    if (_child->y < y+(s>>1))
    {
      i=1;
    }
  }

  child[i]=_child;

  // take data from child as data of this node

  n=_child->n;
  f=_child->f;
  w=_child->w;
}

IDWNode::~IDWNode()
{
  delete child[0];
  delete child[1];
  delete child[2];
  delete child[3];
}

IDWNode *IDWNode::add(unsigned int _x, unsigned int _y, float _f, float _w)
{
  IDWNode *ret=this;

  // is value covered by this node?

  if (_x >= x && _x < x+s && _y >= y && _y < y+s)
  {
    if (s > 1)
    {
      // check into which child node the data point falls

      int i;
      if (_x < x+(s>>1))
      {
        i=2;
        if (_y < y+(s>>1))
        {
          i=0;
        }
      }
      else
      {
        i=3;
        if (_y < y+(s>>1))
        {
          i=1;
        }
      }

      // add to child node

      if (child[i] != 0)
      {
        child[i]=child[i]->add(_x, _y, _f, _w);
      }
      else
      {
        child[i]=new IDWNode(_x, _y, _f, _w);
      }

      // recalculate data of this node

      n=0;
      f=0;
      w=0;

      for (i=0; i<4; i++)
      {
        if (child[i] != 0)
        {
          const int cn=child[i]->n;

          n+=cn;
          f+=cn*child[i]->f;
          w+=cn*child[i]->w;
        }
      }

      const float scale=1.0f/n;

      f*=scale;
      w*=scale;
    }
    else
    {
      // replace data of node with given data

      f=_f;
      w=_w;
    }
  }
  else
  {
    // compute size of parent node that covers existing and new data

    int i=-1;
    unsigned int k=s;
    while (k > 0)
    {
      k>>=1;
      i++;
    }

    i++;
    while (_x < (x>>i)<<i || _x >= ((x>>i)+1)<<i || _y < (y>>i)<<i || _y >= ((y>>i)+1)<<i)
      i++;

    // create parent node

    ret=new IDWNode((x>>i)<<i, (y>>i)<<i, 1<<i, this);

    // add new data value

    ret=ret->add(_x, _y, _f, _w);
  }

  return ret;
}

void IDWNode::get(unsigned int _x, unsigned int _y, float &nominator, float &denominator) const
{
  const float xd=_x-xs;
  const float yd=_y-ys;
  float l2=xd*xd+yd*yd;

  // if individual value or is this node far away?

  if (n == 1 || l2 >= r2)
  {
    if (l2 > 0)
    {
      // interpolate from this node

      float v=w*n/l2;
      nominator+=v*f;
      denominator+=v;
    }
    else
    {
      // if requested coordinate matches coordinate of data value, then just
      // take the data value

      nominator=1e6f*f;
      denominator=1e6;
    }
  }
  else
  {
    // interpolate from all children individually

    for (int i=0; i<4; i++)
    {
      if (child[i] != 0)
      {
        child[i]->get(_x, _y, nominator, denominator);
      }
    }
  }
}

IDW::IDW(int _id)
{
  id=_id;
  root=0;
}

IDW::~IDW()
{
  delete root;
}

void IDW::add(unsigned int x, unsigned int y, float f, float w)
{
  if (root == 0)
  {
    root=new IDWNode(x, y, f, w);
  }
  else
  {
    root=root->add(x, y, f, w);
  }
}

int IDW::getCount() const
{
  if (root != 0)
  {
    return root->getCount();
  }

  return 0;
}

float IDW::get(unsigned int x, unsigned int y) const
{
  if (root != 0)
  {
    float nominator=0;
    float denominator=0;

    root->get(x, y, nominator, denominator);

    if (denominator != 0)
    {
      return nominator/denominator;
    }
  }

  return 0;
}

float IDW::getMean() const
{
  if (root != 0)
  {
    return root->getNodeValue();
  }

  return 0;
}

}
