/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2016 Roboception GmbH
 * Copyright (c) 2014 Institute of Robotics and Mechatronics, German Aerospace Center
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

#include "io.h"
#include "pnm_io.h"
#include "raw_io.h"

#ifdef INCLUDE_GDAL
#include "gdal_io.h"
#endif

#ifdef INCLUDE_JPEG
#include "jpeg_io.h"
#endif

#ifdef INCLUDE_PNG
#include "png_io.h"
#endif

#include <gutil/misc.h>

#include <limits>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <set>
#include <vector>
#include <cctype>
#include <cstdlib>
#include <valarray>

namespace gimage
{

void BasicImageIO::loadHeader(const char *name, long &width, long &height,
  int &depth) const
{
    throw gutil::IOException("Loading the header of this image type is not implemented!"+std::string(name)+")");
}

void BasicImageIO::loadProperties(gutil::Properties &prop, const char *name) const
{
    std::string s=name;
    size_t pos;

    pos=s.rfind(':');
    if (pos != s.npos && s.compare(pos, 2, ":\\") == 0)
      pos=s.npos;

    if (pos == s.npos)
      pos=s.rfind('.');

    if (pos != s.npos)
      s.replace(pos, s.size()-pos, ".hdr");
    else
      s.append(".hdr");

    std::ifstream in;

    in.open(s.c_str(), std::ifstream::in);
    in.close();

    if (!in.fail())
      prop.load(s.c_str());
}

void BasicImageIO::load(ImageU8 &image, const char *name, int ds, long x,
  long y, long w, long h) const
{
    throw gutil::IOException("Loading this image type is not implemented! ("+std::string(name)+")");
}

void BasicImageIO::load(ImageU16 &image, const char *name, int ds, long x,
  long y, long w, long h) const
{
    ImageU8 imageu8;

    load(imageu8, name, ds, x, y, w, h);
    image.setImageLimited(imageu8);
}

void BasicImageIO::load(ImageFloat &image, const char *name, int ds, long x,
  long y, long w, long h) const
{
    ImageU16 imageu16;

    load(imageu16, name, ds, x, y, w, h);
    image.setImageLimited(imageu16);
}

void BasicImageIO::saveProperties(const gutil::Properties &prop, const char *name) const
{
    if (!prop.isEmpty())
    {
      std::string s=name;
      size_t pos;

      pos=s.find(':');
      if (pos != s.npos && s.compare(pos, 2, ":\\") == 0)
        pos=s.npos;

      if (pos == s.npos)
        pos=s.rfind('.');

      if (pos != s.npos)
        s.replace(pos, s.size()-pos, ".hdr");
      else
        s.append(".hdr");

      prop.save(s.c_str(), ("Properties of image "+std::string(name)).c_str());
    }
}

void BasicImageIO::save(const ImageU8 &image, const char *name) const
{
    throw gutil::IOException("Saving this image type is not implemented! ("+std::string(name)+")");
}

void BasicImageIO::save(const ImageU16 &image, const char *name) const
{
    throw gutil::IOException("Saving this image type is not implemented! ("+std::string(name)+")");
}

void BasicImageIO::save(const ImageFloat &image, const char *name) const
{
    throw gutil::IOException("Saving this image type is not implemented! ("+std::string(name)+")");
}

ImageIO::ImageIO()
{
    list.push_back(new PNMImageIO());
    list.push_back(new RAWImageIO());

#ifdef INCLUDE_GDAL
    list.push_back(new GDALImageIO());
#endif

#ifdef INCLUDE_PNG
    list.push_back(new PNGImageIO());
#endif

#ifdef INCLUDE_JPEG
    list.push_back(new JPEGImageIO());
#endif
}

void ImageIO::addBasicImageIO(const BasicImageIO &io)
{
    list.insert(list.begin(), io.create());
}

bool ImageIO::handlesFile(const char *name, bool reading) const
{
    bool ret;

    try
    {
      getBasicImageIO(name, reading);
      ret=true;
    }
    catch (std::exception)
    {
      ret=false;
    }

    return ret;
}

namespace
{

std::string getTileName(const std::string &prefix, int trow, int tcol, const std::string &suffix)
{
    std::ostringstream ret;

    ret.fill('0');
    ret << std::right;

    ret << prefix << '_';

    ret.width(2);
    ret << trow;
    ret.width(0);
    ret << '_';
    ret.width(2);
    ret << tcol;
    ret.width(0);

    ret << '_' << suffix;

    return ret.str();
}

void loadTiledHeader(const BasicImageIO &io, std::set<std::string> &list,
  const std::string &prefix, const std::string &suffix, long &twidth, long &theight,
  long &tborder, long &width, long &height, int &depth)
{
      // initialise return parameters

    list.clear();
    twidth=0;
    theight=0;
    tborder=0;
    width=0;
    height=0;
    depth=0;

      // get optional border size

    gutil::Properties prop;

    try
    {
      prop.load((prefix+".hdr").c_str());
    }
    catch (std::exception)
    { }

    try
    {
      prop.load((prefix+"_param.txt").c_str());
    }
    catch (std::exception)
    { }

    prop.getValue("border", tborder, "0");

      // get list of all tiles

    gutil::getFileList(list, prefix, suffix);
    if (list.size() == 0)
      throw gutil::IOException("There are no tiles: "+prefix+':'+suffix);

      // load header information of first tile

    io.loadHeader((*list.begin()).c_str(), twidth, theight, depth);

      // determine number of rows and columns

    int rows=0;
    int cols=0;
    for (std::set<std::string>::iterator it=list.begin(); it!=list.end(); it++)
    {
      int i, k;
      char sep1, sep2, sep3;
      std::istringstream in(it->substr(prefix.size(), it->size()-prefix.size()-suffix.size()));

      in >> sep1 >> k >> sep2 >> i >> sep3;

      if (in.good() && sep1 == '_' && sep2 == '_' && sep3 == '_')
      {
        cols=std::max(cols, i+1);
        rows=std::max(rows, k+1);
      }
    }

      // compute total size

    width=(twidth-2*tborder)*cols;
    height=(theight-2*tborder)*rows;
}

}

void ImageIO::loadHeader(const char *name, long &width, long &height,
  int &depth) const
{
    std::string s=name;
    size_t pos=s.rfind(':');

    if (pos != s.npos && s.compare(pos, 2, ":\\") == 0)
      pos=s.npos;

    if (pos != s.npos)
    {
        // extract prefix and suffix from file name

      std::string prefix=s.substr(0, pos);
      std::string suffix=s.substr(pos+1);

        // get header information

      std::set<std::string> list;
      long twidth;
      long theight;
      long tborder;

      loadTiledHeader(getBasicImageIO(name, true), list, prefix, suffix, twidth,
        theight, tborder, width, height, depth);
    }
    else
      getBasicImageIO(name, true).loadHeader(name, width, height, depth);
}

namespace
{

template<class T> void loadTiled(const BasicImageIO &io, Image<T> &image,
  const char *name, int ds, long x, long y, long w, long h)
{
    std::string s=name;
    size_t pos=s.rfind(':');

    if (pos != s.npos && s.compare(pos, 2, ":\\") == 0)
      pos=s.npos;

      // extract prefix and suffix from file name

    std::string prefix=s.substr(0, pos);
    std::string suffix=s.substr(pos+1);

      // get information about the tiled image

    std::set<std::string> list;
    long twidth, cwidth;
    long theight, cheight;
    long tborder;
    long width;
    long height;
    int  depth;

    loadTiledHeader(io, list, prefix, suffix, twidth, theight, tborder, width,
      height, depth);

    if (w <= 0)
      w=(width+ds-1)/ds;

    if (h <= 0)
      h=(height+ds-1)/ds;

    cwidth=twidth-2*tborder;
    cheight=theight-2*tborder;

    if (ds == 1 && tborder == 0) // loading without downscaling and overlapping tiles
    {
        // determine block of tiles that is involved

      int tx1=std::max(0l, x)/cwidth;
      int ty1=std::max(0l, y)/cheight;
      int tx2=(x+w-1)/cwidth;
      int ty2=(y+h-1)/cheight;

        // go through all tiles

      image.setSize(w, h, depth);
      image.clear();

      for (int ty=ty1; ty<=ty2; ty++)
      {
        for (int tx=tx1; tx<=tx2; tx++)
        {
            // load part of tile

          long px=std::max(0l, x-tx*cwidth);
          long py=std::max(0l, y-ty*cheight);
          long pw=std::min(twidth-px, x+w-px-tx*cwidth);
          long ph=std::min(theight-py, y+h-py-ty*cheight);

          std::string   tname=getTileName(prefix, ty, tx, suffix);
          Image<T> timage(pw, ph, depth);

          if (list.find(tname) != list.end())
          {
            io.load(timage, tname.c_str(), 1, px, py, pw, ph);

              // copy part of tile with downscaling

            for (long k=0; k<timage.getHeight(); k++)
            {
              const long yy=ty*cheight+py+k-y;

              for (long i=0; i<timage.getWidth(); i++)
              {
                const long xx=tx*cwidth+px+i-x;

                for (int d=0; d<depth; d++)
                  image.set(xx, yy, d, timage.get(i, k, d));
              }
            }
          }
        }
      }
    }
    else
    {
        // determine block of tiles that is involved

      int tx1=std::max(0l, x*ds-tborder)/cwidth;
      int ty1=std::max(0l, y*ds-tborder)/cheight;
      int tx2=((x+w-1)*ds+tborder)/cwidth;
      int ty2=((y+h-1)*ds+tborder)/cheight;

        // go through all tiles

      std::valarray<float> value(w*h*depth);
      std::valarray<float> count(w*h);

      for (int ty=ty1; ty<=ty2; ty++)
      {
        for (int tx=tx1; tx<=tx2; tx++)
        {
            // load part of tile

          long px=std::max(0l, x*ds-(tx*cwidth-tborder));
          long py=std::max(0l, y*ds-(ty*cheight-tborder));
          long pw=std::min(twidth-px, (x+w)*ds-px-(tx*cwidth-tborder));
          long ph=std::min(theight-py, (y+h)*ds-py-(ty*cheight-tborder));

          std::string   tname=getTileName(prefix, ty, tx, suffix);
          Image<T> timage(pw, ph, depth);

          if (list.find(tname) != list.end())
          {
            io.load(timage, tname.c_str(), 1, px, py, pw, ph);

              // copy part of tile with downscaling

            for (long k=0; k<timage.getHeight(); k++)
            {
              for (long i=0; i<timage.getWidth(); i++)
              {
                if (timage.isValid(i, k))
                {
                  long xx=(tx*cwidth-tborder+px+i-x*ds)/ds;
                  long yy=(ty*cheight-tborder+py+k-y*ds)/ds;

                  if (tborder > 0)
                  {
                    float cx=std::min((px+i)/(2.0f*tborder), (twidth-px-i)/(2.0f*tborder));
                    float cy=std::min((py+k)/(2.0f*tborder), (theight-py-k)/(2.0f*tborder));

                    cx=std::max(1e-6f, std::min(1.0f, cx));
                    cy=std::max(1e-6f, std::min(1.0f, cy));

                    float c=cx*cy;

                    for (int d=0; d<depth; d++)
                      value[(d*h+yy)*w+xx]+=c*static_cast<float>(timage.get(i, k, d));

                    count[yy*w+xx]+=c;
                  }
                  else
                  {
                    for (int d=0; d<depth; d++)
                      value[(d*h+yy)*w+xx]+=static_cast<float>(timage.get(i, k, d));

                    count[yy*w+xx]++;
                  }
                }
              }
            }
          }
        }
      }

        // copy result to target image

      image.setSize(w, h, depth);
      image.clear();

      for (long k=0; k<h; k++)
      {
        for (long i=0; i<w; i++)
        {
          float c=count[k*w+i];
          if (c > 0)
          {
            for (int d=0; d<depth; d++)
              image.set(i, k, d, static_cast<typename Image<T>::store_t>(value[(d*h+k)*w+i]/c));
          }
        }
      }
    }
}

}

void ImageIO::loadProperties(gutil::Properties &prop, const char *name) const
{
    getBasicImageIO(name, true).loadProperties(prop, name);
}

void ImageIO::load(ImageU8 &image, const char *name, int ds, long x, long y,
  long w, long h) const
{
    std::string s=name;
    size_t pos=s.rfind(':');

    if (pos != s.npos && s.compare(pos, 2, ":\\") == 0)
      pos=s.npos;

    if (pos == s.npos)
      getBasicImageIO(name, true).load(image, name, ds, x, y, w, h);
    else
      loadTiled(getBasicImageIO(name, true), image, name, ds, x, y, w, h);
}

void ImageIO::load(ImageU16 &image, const char *name, int ds, long x, long y,
  long w, long h) const
{
    std::string s=name;
    size_t pos=s.rfind(':');

    if (pos != s.npos && s.compare(pos, 2, ":\\") == 0)
      pos=s.npos;

    if (pos == s.npos)
      getBasicImageIO(name, true).load(image, name, ds, x, y, w, h);
    else
      loadTiled(getBasicImageIO(name, true), image, name, ds, x, y, w, h);
}

void ImageIO::load(ImageFloat &image, const char *name, int ds, long x, long y,
  long w, long h) const
{
    std::string s=name;
    size_t pos=s.rfind(':');

    if (pos != s.npos && s.compare(pos, 2, ":\\") == 0)
      pos=s.npos;

    if (pos == s.npos)
      getBasicImageIO(name, true).load(image, name, ds, x, y, w, h);
    else
      loadTiled(getBasicImageIO(name, true), image, name, ds, x, y, w, h);
}

void ImageIO::saveProperties(const gutil::Properties &prop, const char *name) const
{
    getBasicImageIO(name, false).saveProperties(prop, name);
}

void ImageIO::save(const ImageU8 &image, const char *name) const
{
    getBasicImageIO(name, false).save(image, name);
}

void ImageIO::save(const ImageU16 &image, const char *name) const
{
    getBasicImageIO(name, false).save(image, name);
}

void ImageIO::save(const ImageFloat &image, const char *name) const
{
    getBasicImageIO(name, false).save(image, name);
}

const BasicImageIO &ImageIO::getBasicImageIO(const char *name, bool reading) const
{
    for (std::vector<BasicImageIO *>::const_iterator it=list.begin(); it<list.end(); it++)
    {
      if ((*it)->handlesFile(name, reading))
        return **it;
    }

    throw gutil::IOException("Unknown image type ("+std::string(name)+")");
}

ImageIO& getImageIO()
{
    static ImageIO *io=new ImageIO();
    return *io;
}

std::string getNewImageName(std::string prefix)
{
      // try getting existing suffix

    std::string suffix;

    size_t i=prefix.rfind('.');
    if (i != prefix.npos && prefix.size()-i <= 4)
    {
      suffix=prefix.substr(i);
      prefix=prefix.substr(0, i);
    }

      // determine supported format for writing image

    if (suffix.size() == 0)
    {
      suffix=".ppm";
      if (getImageIO().handlesFile("file.png", false))
        suffix=".png";
      else if (getImageIO().handlesFile("file.tif", false))
        suffix=".tif";
    }

      // find file name that is not used

    int c=0;
    std::string name;

    while (name.size() == 0 && c < 1000)
    {
      std::ostringstream out;
      out << prefix << "_";

      if (c < 100)
        out << "0";

      if (c < 10)
        out << "0";

      out << c++ << suffix;

      std::ifstream file(out.str().c_str());

      if (!file.is_open())
        name=out.str();

      file.close();
    }

    return name;
}

}