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

#ifndef GIMAGE_HDR_H
#define GIMAGE_HDR_H

#include "gauss_pyramid.h"
#include "laplace_pyramid.h"

#include "io.h"

#include <vector>
#include <memory>
#include <type_traits>

namespace gimage
{

class HighDynamicRangeFusionBase
{
  protected:

    std::vector<float> gauss;

    struct Data
    {
      std::vector<ImageFloat> laplace;
      std::vector<ImageFloat> weight;
    };

    std::vector<std::shared_ptr<Data> > list;

    HighDynamicRangeFusionBase();

    void setContrastWeight(ImageFloat &wp, const ImageFloat &lp, float max_value);

    void normalizeWeights(float scale);

    void mulWeight(ImageFloat &target, ImageFloat &weight);

    void addMulWeight(ImageFloat &target, ImageFloat &image, ImageFloat &weight);
};

/**
  Performs high dynamic range fusion of the given list of images which must all
  have the same size and depth.

  See: Mertens, T., Kautz, J. and Van Reeth, F., "Exposure Fusion", in
  Proceedings of Pacific Graphics 2007.
*/

template<class T> class HighDynamicRangeFusion : private HighDynamicRangeFusionBase
{
  private:

    bool saturation;
    bool contrast;
    bool well_exposedness;

    void mulWellExposednessWeight(ImageFloat &wp, const Image<T> &image, float max_value)
    {
      float *p=wp.getPtr(0, 0, 0);
      for (long k=0; k<image.getHeight(); k++)
      {
        for (long i=0; i<image.getWidth(); i++)
        {
          float v=1;
          for (int d=0; d<image.getDepth(); d++)
          {
            T pv=std::max(static_cast<T>(0), std::min(static_cast<T>(max_value),
              image.get(i, k, d)));

            v*=gauss[static_cast<size_t>(gauss.size()*pv/max_value)];
          }

          // multiply distance from center intensity, weighted by Gauss

          *p++ *= v;
        }
      }
    }

    void mulWellExposednessWeightU8(ImageFloat &wp, const Image<T> &image)
    {
      float *p=wp.getPtr(0, 0, 0);
      for (long k=0; k<image.getHeight(); k++)
      {
        for (long i=0; i<image.getWidth(); i++)
        {
          float v=1;
          for (int d=0; d<image.getDepth(); d++)
          {
            v*=gauss[image.get(i, k, d)];
          }

          // multiply distance from center intensity, weighted by Gauss

          *p++ *= v;
        }
      }
    }

    void mulSaturationWeight(ImageFloat &wp, const Image<T> &image, float max_value)
    {
      float scale=1.0f/max_value;
      int n=image.getDepth();

      float *p=wp.getPtr(0, 0, 0);
      for (long k=0; k<image.getHeight(); k++)
      {
        for (long i=0; i<image.getWidth(); i++)
        {
          float ssum=0;
          float mean=0;

          for (int d=0; d<n; d++)
          {
            float v=scale*image.get(i, k, d);
            ssum+=v*v;
            mean+=v;
          }

          mean/=n;

          // multiply standard deviation of color values

          *p++ *= static_cast<float>(std::sqrt(std::max(0.0f, (ssum-n*mean*mean)/n)));
        }
      }
    }

  public:

    HighDynamicRangeFusion()
    {
      // initialize parameters

      saturation=true;
      contrast=true;
      well_exposedness=true;
    }

    /**
      Removes all added images and chooses what features to prefer for
      weighting.

      @param saturation       Prefer well saturated colors.
      @param contrast         Prefer high contrast.
      @param well_exposedness Prefer well exposed values.
    */

    void clear(bool _saturation=true, bool _contrast=true, bool _well_exposedness=true)
    {
      list.clear();

      saturation=_saturation;
      contrast=_contrast;
      well_exposedness=_well_exposedness;
    }

    /**
      Adds an image. All added images must have the same size and color depth
      and must capture the same scene from the same viewpoint, but they should
      be taken with different exposure times and gain values.

      @param image     Input image.
      @param max_value Maximum pixel value, used for scaling weighting functions.
    */

    void add(const Image<T> &image, float max_value=255)
    {
      // check that image has same size and depth as previous images

      if (list.size() > 0 && (list[0]->laplace[0].getWidth() != image.getWidth() ||
          list[0]->laplace[0].getHeight() != image.getHeight() ||
          list[0]->laplace[0].getDepth() != image.getDepth()))
      {
        throw std::invalid_argument("HighDynamicRangeFusion::add(): All images must have same size and depth");
      }

      // create Laplace pyramid from image

      std::shared_ptr<Data> p=std::make_shared<Data>();
      list.push_back(p);

      createLaplacianPyramid(p->laplace, image);

      // compute weights for highest resolution

      p->weight.resize(p->laplace.size());
      p->weight[0].setSize(image.getWidth(), image.getHeight(), 1);

      // weight according to local contrast

      if (contrast)
      {
        setContrastWeight(p->weight[0], p->laplace[0], max_value);
      }
      else
      {
        p->weight[0]=1;
      }

      // weight according to exposedness, i.e. not under- or overexposed

      if (well_exposedness)
      {
        if (std::is_same<T, gutil::uint8>::value && max_value == 255)
        {
          mulWellExposednessWeightU8(p->weight[0], image);
        }
        else
        {
          mulWellExposednessWeight(p->weight[0], image, max_value);
        }
      }

      // if color image, weight according to saturation

      if (saturation && image.getDepth() > 1)
      {
        mulSaturationWeight(p->weight[0], image, max_value);
      }
    }

    /**
      Performs fusion and clears all added images.

      @param image Output image.
      @param scale Factor for scaling pixel values into target image. Values
                   that do not fit into the target image range will be clipped.
    */

    void fuse(Image<T> &image, float scale=1.0f)
    {
      // normalize weights

      normalizeWeights(scale);

      // for all input images

      for (size_t j=0; j<list.size(); j++)
      {
        // compute Gaussian pyramid for weights

        createGaussPyramid(list[j]->weight, list[j]->weight[0]);

        // perform weighted multi resolution blending into first laplace
        // pyramid (and free weight and laplace pyramids, except first one)

        if (j == 0)
        {
          for (size_t l=0; l<list[0]->weight.size(); l++)
          {
            mulWeight(list[0]->laplace[l], list[0]->weight[l]);
          }

          list[0]->weight.clear();
        }
        else
        {
          for (size_t l=0; l<list[j]->weight.size(); l++)
          {
            addMulWeight(list[0]->laplace[l], list[j]->laplace[l], list[j]->weight[l]);
          }

          list[j].reset();
        }
      }

      // collapse resulting laplacian pyramid into result image

      collapseLaplacianPyramid(image, list[0]->laplace);

      // clean intermediate data

      list.clear();
    }
};

}

#endif
