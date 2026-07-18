/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2026 Roboception GmbH
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

#include "segmentation.h"

#include <vector>

namespace gimage
{

namespace
{

// Number of possible barrier costs for an uint8 image, i.e. the barrier is in
// the range [0, 255].

const int BARRIER_LEVELS=256;

/**
  Bucket (Dial) priority queue for non-negative integer costs in the range
  [0, BARRIER_LEVELS-1]. Pixels are stored by their linear index. As the
  wavefront never relaxes a neighbour to a cost below the cost of the pixel it
  is expanded from, the read position only moves forward and the queue works in
  linear time.
*/

class BucketQueue
{
  private:

    std::vector<std::vector<long> > bucket;
    int                             current;

  public:

    BucketQueue() : bucket(BARRIER_LEVELS), current(0) { }

    void push(long index, int cost)
    {
      bucket[cost].push_back(index);
    }

    // Returns the linear index of the pixel with the smallest cost and reports
    // that cost, or -1 if the queue is empty.

    long pop(int &cost)
    {
      while (current < BARRIER_LEVELS && bucket[current].empty())
      {
        current++;
      }

      if (current >= BARRIER_LEVELS)
      {
        return -1;
      }

      long index=bucket[current].back();
      bucket[current].pop_back();
      cost=current;

      return index;
    }
};

// Absolute difference of two pixels, taken as the largest absolute difference
// over all channels.

inline int channelBarrier(const ImageU8 &image, long i0, long k0, long i1, long k1)
{
  int ret=0;

  for (int j=0; j<image.getDepth(); j++)
  {
    int d=static_cast<int>(image.get(i0, k0, j))-static_cast<int>(image.get(i1, k1, j));

    if (d < 0)
    {
      d=-d;
    }

    ret=std::max(ret, d);
  }

  return ret;
}

// Union-find with path compression and union by size. The elements are the
// linear pixel indices.

class UnionFind
{
  private:

    std::vector<long> parent;
    std::vector<long> size;

  public:

    explicit UnionFind(long n) : parent(n), size(n, 1)
    {
      for (long i=0; i<n; i++)
      {
        parent[i]=i;
      }
    }

    long find(long i)
    {
      while (parent[i] != i)
      {
        parent[i]=parent[parent[i]]; // halve the path
        i=parent[i];
      }

      return i;
    }

    long getSize(long i)
    {
      return size[find(i)];
    }

    void merge(long a, long b)
    {
      a=find(a);
      b=find(b);

      if (a != b)
      {
        if (size[a] < size[b])
        {
          std::swap(a, b);
        }

        parent[b]=a;
        size[a]+=size[b];
      }
    }
};

}

void barrierSegmentation(ImageU32 &seg, const ImageU8 &image,
  const std::vector<Seed> &seeds)
{
  const long width=image.getWidth();
  const long height=image.getHeight();
  const int  depth=image.getDepth();
  const long n=width*height;

  seg.setSize(width, height, 1);
  seg=0;

  if (n <= 0)
  {
    return;
  }

  // barrier cost per pixel (initialised to an unreachable value) as well as the
  // running minimum and maximum intensity of the best path per channel

  std::vector<int>           cost(n, BARRIER_LEVELS);
  std::vector<gutil::uint8>  lo(n*depth);
  std::vector<gutil::uint8>  hi(n*depth);

  gutil::uint32 *label=seg.getPtr(0, 0);

  BucketQueue queue;

  // insert all seeds with barrier 0

  for (size_t s=0; s<seeds.size(); s++)
  {
    const Seed &seed=seeds[s];

    if (seed.x < 0 || seed.x >= width || seed.y < 0 || seed.y >= height)
    {
      continue;
    }

    long index=seed.y*width+seed.x;

    if (cost[index] == 0) // position already occupied by an earlier seed
    {
      continue;
    }

    cost[index]=0;
    label[index]=seed.label;

    for (int j=0; j<depth; j++)
    {
      gutil::uint8 v=image.get(seed.x, seed.y, j);
      lo[index*depth+j]=v;
      hi[index*depth+j]=v;
    }

    queue.push(index, 0);
  }

  // propagate the wavefront over the 4-connected neighbourhood

  const long neighbour[4]={-1, 1, -width, width};

  int c;
  long index;

  while ((index=queue.pop(c)) >= 0)
  {
    if (c != cost[index]) // outdated entry, already expanded with lower cost
    {
      continue;
    }

    long i=index%width;
    long k=index/width;

    for (int d=0; d<4; d++)
    {
      // reject neighbours that leave the image

      if ((d == 0 && i == 0) || (d == 1 && i == width-1) ||
          (d == 2 && k == 0) || (d == 3 && k == height-1))
      {
        continue;
      }

      long nindex=index+neighbour[d];
      long ni=nindex%width;
      long nk=nindex/width;

      // extend the path of the current pixel to the neighbour and determine
      // the barrier as the largest range over all channels

      int ncost=0;

      for (int j=0; j<depth; j++)
      {
        int v=static_cast<int>(image.get(ni, nk, j));
        int l=std::min(static_cast<int>(lo[index*depth+j]), v);
        int h=std::max(static_cast<int>(hi[index*depth+j]), v);
        ncost=std::max(ncost, h-l);
      }

      if (ncost < cost[nindex])
      {
        cost[nindex]=ncost;
        label[nindex]=label[index];

        for (int j=0; j<depth; j++)
        {
          int v=static_cast<int>(image.get(ni, nk, j));
          lo[nindex*depth+j]=static_cast<gutil::uint8>(std::min(static_cast<int>(lo[index*depth+j]), v));
          hi[nindex*depth+j]=static_cast<gutil::uint8>(std::max(static_cast<int>(hi[index*depth+j]), v));
        }

        queue.push(nindex, ncost);
      }
    }
  }
}

gutil::uint32 barrierSegmentation(ImageU32 &seg, const ImageU8 &image,
  int threshold, long minsize)
{
  const long width=image.getWidth();
  const long height=image.getHeight();
  const long n=width*height;

  seg.setSize(width, height, 1);
  seg=0;

  if (n <= 0)
  {
    return 0;
  }

  UnionFind uf(n);

  // merge every pixel with its right and lower neighbour if the barrier across
  // the border is within the threshold

  for (long k=0; k<height; k++)
  {
    for (long i=0; i<width; i++)
    {
      long index=k*width+i;

      if (i+1 < width && channelBarrier(image, i, k, i+1, k) <= threshold)
      {
        uf.merge(index, index+1);
      }

      if (k+1 < height && channelBarrier(image, i, k, i, k+1) <= threshold)
      {
        uf.merge(index, index+width);
      }
    }
  }

  // remove segments below the minimum size by merging them into the
  // neighbouring segment across the border with the smallest barrier

  if (minsize > 1)
  {
    // collect all borders sorted by their barrier via bucket sort; a border is
    // encoded as 2*index (+0 for the right and +1 for the lower neighbour)

    std::vector<std::vector<long> > border(BARRIER_LEVELS);

    for (long k=0; k<height; k++)
    {
      for (long i=0; i<width; i++)
      {
        long index=k*width+i;

        if (i+1 < width)
        {
          border[channelBarrier(image, i, k, i+1, k)].push_back(2*index);
        }

        if (k+1 < height)
        {
          border[channelBarrier(image, i, k, i, k+1)].push_back(2*index+1);
        }
      }
    }

    for (int b=0; b<BARRIER_LEVELS; b++)
    {
      for (size_t e=0; e<border[b].size(); e++)
      {
        long index=border[b][e]>>1;
        long nindex=(border[b][e]&1) ? index+width : index+1;

        if (uf.find(index) != uf.find(nindex) &&
            (uf.getSize(index) < minsize || uf.getSize(nindex) < minsize))
        {
          uf.merge(index, nindex);
        }
      }
    }
  }

  // assign a consecutive label starting at 1 to every segment root

  std::vector<gutil::uint32> rootlabel(n, 0);
  gutil::uint32 *label=seg.getPtr(0, 0);
  gutil::uint32 count=0;

  for (long index=0; index<n; index++)
  {
    long root=uf.find(index);

    if (rootlabel[root] == 0)
    {
      rootlabel[root]=++count;
    }

    label[index]=rootlabel[root];
  }

  return count;
}

}
