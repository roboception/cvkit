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

#ifndef GIMAGE_SEGMENTATION_H
#define GIMAGE_SEGMENTATION_H

#include "image.h"

#include <vector>

namespace gimage
{

/**
  A seed for the seeded barrier segmentation. It defines the label that is
  assigned to the pixel at the given position and, by propagation, to all
  pixels that are reached from this seed with the smallest barrier.
*/

struct Seed
{
  long          x;      // column of the seed pixel
  long          y;      // row of the seed pixel
  gutil::uint32 label;  // label > 0 that is assigned to this seed

  Seed() : x(0), y(0), label(0) { }
  Seed(long _x, long _y, gutil::uint32 _label) : x(_x), y(_y), label(_label) { }
};

/**
  Seeded segmentation based on the minimum barrier distance.

  The barrier of a path is the difference between the maximum and the minimum
  intensity along the path. For multi channel images it is the largest barrier
  over all channels. Starting from the given seeds all pixels are labelled
  simultaneously: every pixel receives the label of the seed that reaches it
  with the smallest barrier. Ties are resolved in favour of the seed that is
  processed first.

  The propagation is computed with an image foresting transform (i.e. a
  Dijkstra like wavefront over a 4-connected neighbourhood), which yields the
  commonly used approximation of the exact minimum barrier distance.

  @param seg   Output label image. It is resized to the size of the input
               image and set to the store type uint32. Pixels carry the label
               of the seed they belong to. Pixels that cannot be reached from
               any seed (e.g. if no seeds are given) are set to 0.
  @param image Input intensity image with one or more channels.
  @param seeds List of seeds with their labels. Seeds outside the image are
               ignored. If two seeds share a position, the first one wins.
*/

void barrierSegmentation(ImageU32 &seg, const ImageU8 &image,
  const std::vector<Seed> &seeds);

/**
  Segmentation that treats every pixel as its own seed and merges neighbouring
  regions into segments.

  Every pixel starts as an individual region. Two 4-connected pixels are merged
  as long as the barrier across their common border, i.e. the absolute
  intensity difference (the largest over all channels for multi channel
  images), does not exceed the given threshold. The resulting segments are the
  connected components of this relation and correspond to cutting the barrier
  merge hierarchy at the given threshold.

  Optionally, segments that are smaller than minsize are removed in a
  subsequent step by merging them into the neighbouring segment across the
  border with the smallest barrier, regardless of the threshold. This is
  repeated implicitly until no segment below minsize remains (except a possible
  single segment that covers the whole image).

  @param seg       Output label image. It is resized to the size of the input
                   image and set to the store type uint32. Each segment is
                   labelled with a unique value starting at 1.
  @param image     Input intensity image with one or more channels.
  @param threshold Maximum intensity difference across a border for which two
                   neighbouring pixels are placed into the same segment.
  @param minsize   Minimum number of pixels per segment. Smaller segments are
                   merged into a neighbour. A value <= 1 disables this step.
  @return          Number of segments (i.e. the largest assigned label).
*/

gutil::uint32 barrierSegmentation(ImageU32 &seg, const ImageU8 &image,
  int threshold, long minsize=0);

}

#endif
