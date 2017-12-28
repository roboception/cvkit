/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2014, Institute of Robotics and Mechatronics, German Aerospace Center
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

#ifndef BGUI_IMAGEADAPTER_H
#define BGUI_IMAGEADAPTER_H

#include "imageadapterbase.h"

#include <gutil/exception.h>
#include <gimage/size.h>

#include <cmath>
#include <limits>

namespace bgui
{

template<class T> class ImageAdapter : public ImageAdapterBase
{
  private:

    const gimage::Image<T>         *image;
    bool                           del;
    double                         vmin, vmax;
    std::vector<gimage::Image<T> > mipmap;

  public:

    ImageAdapter(const gimage::Image<T> *im, double valid_min=-std::numeric_limits<float>::max(),
                 double valid_max=std::numeric_limits<float>::max(), bool delete_on_close=false)
    {
      image=im;
      del=delete_on_close;

      vmin=valid_min;
      vmax=valid_max;

      adaptMinMaxIntensity(0, 0, image->getWidth(), image->getHeight());
    }

    virtual ~ImageAdapter()
    {
      if (del)
      {
        delete image;
      }
    }

    void setSmoothing(bool tf)
    {
      if (tf && mipmap.size() == 0)
      {
        const gimage::Image<T> *p=image;
        long w=p->getWidth();
        long h=p->getHeight();

        while (w > 64 && h > 64)
        {
          mipmap.push_back(downscaleImage(*p, 2));

          p=&mipmap.back();
          w=p->getWidth();
          h=p->getHeight();
        }
      }
      else if (!tf)
      {
        mipmap.clear();
      }
    }

    bool getSmoothing()
    {
      return mipmap.size() > 0;
    }

    void adaptMinMaxIntensity(long x, long y, long w, long h)
    {
      // compute original area

      const gmath::SVector<long, 2> p1=R*gmath::SVector<long, 3>(static_cast<long>(x/scale),
                                       static_cast<long>(y/scale), 1);
      const gmath::SVector<long, 2> p2=R*gmath::SVector<long, 3>(static_cast<long>((x+w-1)/scale),
                                       static_cast<long>((y+h-1)/scale), 1);

      long xmin=std::min(p1[0], p2[0]), xmax=std::max(p1[0], p2[0]);
      long ymin=std::min(p1[1], p2[1]), ymax=std::max(p1[1], p2[1]);

      xmin=std::max(xmin, 0l);
      xmax=std::min(xmax, image->getWidth()-1);
      ymin=std::max(ymin, 0l);
      ymax=std::min(ymax, image->getHeight()-1);

      // search minimum and maximum valid intensity in the specified area

      imin=vmax;
      imax=vmin;

      int dmin=0;
      int dmax=image->getDepth()-1;

      if (channel >= 0 && channel < image->getDepth())
      {
        dmin=dmax=channel;
      }

      for (int d=dmin; d<=dmax; d++)
      {
        for (long y=ymin; y<ymax; y++)
        {
          for (long x=xmin; x<xmax; x++)
          {
            double v=image->get(x, y, d);

            if (v >= vmin && v <= vmax)
            {
              imin=std::min(imin, v);
              imax=std::max(imax, v);
            }
          }
        }
      }

      if (imax <= imin)
      {
        imin=0;
        imax=1;
      }
    }

    long getOriginalWidth() const
    {
      return image->getWidth();
    }

    long getOriginalHeight() const
    {
      return image->getHeight();
    }

    int getOriginalDepth() const
    {
      return image->getDepth();
    }

    std::string getOriginalType() const
    {
      return std::string(image->getTypeDescription());
    }

    double getIntensityOfPixel(long x, long y) const
    {
      double ret=0;

      x=static_cast<long>(x/scale);
      y=static_cast<long>(y/scale);

      gmath::SVector<long, 2> p=R*gmath::SVector<long, 3>(x, y, 1);

      int dmin=0;
      int dmax=image->getDepth()-1;

      if (channel >= 0 && channel < image->getDepth())
      {
        dmin=dmax=channel;
      }

      for (int d=dmin; d<=dmax; d++)
      {
        if (image->get(p[0], p[1], d) > ret)
        {
          ret=image->get(p[0], p[1], d);
        }
      }

      if (ret < vmin || ret > vmax)
      {
        ret=std::numeric_limits<double>::infinity();
      }

      return ret;
    }

    std::string getDescriptionOfPixel(long x, long y) const
    {
      std::ostringstream os;

      x=static_cast<long>(x/scale);
      y=static_cast<long>(y/scale);

      gmath::SVector<long, 2> p=R*gmath::SVector<long, 3>(x, y, 1);

      os << "x=";

      os.width(5);
      os << p[0] << ", y=";

      os.width(5);
      os << p[1] << ":";

      for (int d=0; d<image->getDepth(); d++)
      {
        os << " ";

        os.width(5);
        os << image->getW(p[0], p[1], d);
      }

      return os.str();
    }

    long getWidth() const
    {
      if ((rotation&1) == 0)
      {
        return static_cast<long>(scale*image->getWidth());
      }
      else
      {
        return static_cast<long>(scale*image->getHeight());
      }
    }

    long getHeight() const
    {
      if ((rotation&1) == 0)
      {
        return static_cast<long>(scale*image->getHeight());
      }
      else
      {
        return static_cast<long>(scale*image->getWidth());
      }
    }

    double getPixel(float x, float y, int c) const
    {
      if (mipmap.size() > 0)
      {
        int j=static_cast<int>(log(1/scale)/log(2))-1;
        j=std::min(static_cast<int>(mipmap.size())-1, j);

        int ds=1<<(j+1);
        double ret;

        if (j < 0)
        {
          ret=image->getBilinear(x, y, c);
        }
        else
        {
          ret=mipmap[j].getBilinear(x/ds, y/ds, c);
        }

        if (j+1 >= 0 && j+1 < static_cast<int>(mipmap.size()))
        {
          double f=(1/scale-ds)/ds;

          if (f > 1e-6)
          {
            ret=(1-f)*ret+f*mipmap[j+1].getBilinear(x/(2*ds), y/(2*ds), c);
          }
        }

        return ret;
      }

      return image->get(static_cast<long>(x), static_cast<long>(y), c);
    }

    void getPixel(double &r, double &g, double &b, float x, float y,
                  int ir, int ig, int ib) const
    {
      if (mipmap.size() > 0)
      {
        int j=static_cast<int>(log(1/scale)/log(2))-1;
        j=std::min(static_cast<int>(mipmap.size())-1, j);

        int ds=1<<(j+1);

        if (j < 0)
        {
          r=image->getBilinear(x, y, ir);
          g=image->getBilinear(x, y, ig);
          b=image->getBilinear(x, y, ib);
        }
        else
        {
          r=mipmap[j].getBilinear(x/ds, y/ds, ir);
          g=mipmap[j].getBilinear(x/ds, y/ds, ig);
          b=mipmap[j].getBilinear(x/ds, y/ds, ib);
        }

        if (j+1 >= 0 && j+1 < static_cast<int>(mipmap.size()))
        {
          double f=(1/scale-ds)/ds;

          if (f > 1e-6)
          {
            r=(1-f)*r+f*mipmap[j+1].getBilinear(x/(2*ds), y/(2*ds), ir);
            g=(1-f)*g+f*mipmap[j+1].getBilinear(x/(2*ds), y/(2*ds), ig);
            b=(1-f)*b+f*mipmap[j+1].getBilinear(x/(2*ds), y/(2*ds), ib);
          }
        }

        return;
      }

      r=image->get(static_cast<long>(x), static_cast<long>(y), ir);
      g=image->get(static_cast<long>(x), static_cast<long>(y), ig);
      b=image->get(static_cast<long>(x), static_cast<long>(y), ib);
    }

    void copyInto(gimage::ImageU8 &rgb, long x, long y) const
    {
      const double step=1/scale;
      const double irange=imax-imin;
      int ir=0, ig=1, ib=2;
      long iw=image->getWidth();
      long ih=image->getHeight();
      double rainbow_size=1;

      if (irange > 0)
      {
        rainbow_size=std::pow(10, std::floor(std::log(irange)/std::log(10))-1);
      }

      if ((rotation&1) != 0)
      {
        iw=image->getHeight();
        ih=image->getWidth();
      }

      assert(rgb.getDepth() == 3);

      if (image->getDepth() < 3)
      {
        ir=ig=ib=0;
      }

      if (channel >= 0 && channel < image->getDepth())
      {
        ir=ig=ib=channel;
      }

      for (int k=0; k<rgb.getHeight(); k++)
      {
        double yf=(y+k)/scale;

        int i=0;

        if (yf >= 0 && yf < ih)
        {
          double xf=x/scale;

          while (xf < 0 && i < rgb.getWidth())
          {
            rgb.set(i, k, 0, 0);
            rgb.set(i, k, 1, 0);
            rgb.set(i, k, 2, 0);

            xf+=step;
            i++;
          }

          if (ig != ir || map == map_raw)
          {
            // map color image or greyscale image directly

            while (xf < iw && i < rgb.getWidth())
            {
              float xs=static_cast<float>(R(0, 0)*xf+R(0, 1)*yf+R(0, 2));
              float ys=static_cast<float>(R(1, 0)*xf+R(1, 1)*yf+R(1, 2));

              double r, g, b;
              getPixel(r, g, b, xs, ys, ir, ig, ib);

              r=(std::max(imin, std::min(imax, r))-imin)/irange;
              g=(std::max(imin, std::min(imax, g))-imin)/irange;
              b=(std::max(imin, std::min(imax, b))-imin)/irange;

              if (gamma != 1.0)
              {
                // pow() can be extremly slow and linear lookup tables would be
                // very big for data with higher radiometric depth, thats why we
                // only allow a gamma with root of 2 and 4.

                if (static_cast<int>(gamma+0.5) == 2)
                {
                  r=sqrt(r);
                  g=sqrt(g);
                  b=sqrt(b);
                }
                else if (static_cast<int>(gamma+0.5) >= 3)
                {
                  r=sqrt(sqrt(r));
                  g=sqrt(sqrt(g));
                  b=sqrt(sqrt(b));
                }
              }

              rgb.set(i, k, 0, static_cast<char>(255*r+0.5));
              rgb.set(i, k, 1, static_cast<char>(255*g+0.5));
              rgb.set(i, k, 2, static_cast<char>(255*b+0.5));

              xf+=step;
              i++;
            }
          }
          else if (map == map_jet)
          {
            // map greyscale image using jet encoding

            while (xf < iw && i < rgb.getWidth())
            {
              float xs=static_cast<float>(R(0, 0)*xf+R(0, 1)*yf+R(0, 2));
              float ys=static_cast<float>(R(1, 0)*xf+R(1, 1)*yf+R(1, 2));

              double v=getPixel(xs, ys, ir);
              double r=0.0;
              double g=0.0;
              double b=0.0;

              if (image->isValid(static_cast<long>(xs), static_cast<long>(ys)))
              {
                if (v >= imin)
                {
                  if (v <= imax)
                  {
                    v=(v-imin)/irange;

                    if (gamma != 1.0)
                    {
                      // pow() can be extremly slow and linear lookup tables would be
                      // very big for data with higher radiometric depth, thats why we
                      // only allow a gamma with root of 2 and 4.

                      if (static_cast<int>(gamma+0.5) == 2)
                      {
                        v=sqrt(v);
                      }
                      else if (static_cast<int>(gamma+0.5) >= 3)
                      {
                        v=sqrt(sqrt(r));
                      }
                    }
                  }
                  else
                  {
                    v=1.05;
                  }
                }
                else
                {
                  v=-0.05;
                }

                v=v/1.15+0.1;
                r=std::max(0.0, std::min(1.0, (1.5 - 4*fabs(v-0.75))));
                g=std::max(0.0, std::min(1.0, (1.5 - 4*fabs(v-0.5))));
                b=std::max(0.0, std::min(1.0, (1.5 - 4*fabs(v-0.25))));
              }

              rgb.set(i, k, 0, static_cast<char>(255*r+0.5));
              rgb.set(i, k, 1, static_cast<char>(255*g+0.5));
              rgb.set(i, k, 2, static_cast<char>(255*b+0.5));

              xf+=step;
              i++;
            }
          }
          else if (map == map_rainbow)
          {
            while (xf < iw && i < rgb.getWidth())
            {
              float xs=static_cast<float>(R(0, 0)*xf+R(0, 1)*yf+R(0, 2));
              float ys=static_cast<float>(R(1, 0)*xf+R(1, 1)*yf+R(1, 2));

              double v=getPixel(xs, ys, ir);
              double r=0.0;
              double g=0.0;
              double b=0.0;

              if (image->isValid(static_cast<long>(xs), static_cast<long>(ys)))
              {
                if (v < imin)
                {
                  v=imin;
                }

                if (v > imax)
                {
                  v=imax;
                }

                // compute color

                double v1=v/rainbow_size;

                v1=6.0*(v1-floor(v1));

                int v0=static_cast<int>(v1);

                v1-=v0;

                switch (v0)
                {
                  case 0:
                    r=1.0;
                    g=v1;
                    break;

                  case 1:
                    r=1.0-v1;
                    g=1.0;
                    break;

                  case 2:
                    g=1.0;
                    b=v1;
                    break;

                  case 3:
                    g=1.0-v1;
                    b=1.0;
                    break;

                  case 4:
                    b=1.0;
                    r=v1;
                    break;

                  default:
                    b=1.0-v1;
                    r=1.0;
                    break;
                }

                // compute brightness

                v=1.6*(v-imin)/irange-0.8;

                if (v < 0)
                {
                  r*=v+1;
                  g*=v+1;
                  b*=v+1;
                }
                else
                {
                  r=(1-v)*r+v;
                  g=(1-v)*g+v;
                  b=(1-v)*b+v;
                }
              }

              rgb.set(i, k, 0, static_cast<int>(255*r+0.5));
              rgb.set(i, k, 1, static_cast<int>(255*g+0.5));
              rgb.set(i, k, 2, static_cast<int>(255*b+0.5));

              xf+=step;
              i++;
            }
          }
        }

        while (i < rgb.getWidth())
        {
          rgb.set(i, k, 0, 0);
          rgb.set(i, k, 1, 0);
          rgb.set(i, k, 2, 0);

          i++;
        }
      }
    }
};

}

#endif
