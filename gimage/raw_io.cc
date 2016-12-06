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

#include "raw_io.h"

#include <gutil/properties.h>

#include <limits>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cctype>
#include <cstdlib>
#include <valarray>

namespace gimage
{

namespace
{

std::string readRAWHeader(const char *name, int &type, bool &msbfirst, long &width,
                          long &height)
{
  std::string s=name;
  size_t pos;
  std::string ret;

  pos=s.find('&');

  if (pos != s.npos)
  {
    std::istringstream in(s.substr(pos+1));
    char sep1=0, sep2=0, order;

    in >> std::noskipws >> width >> sep1 >> height >> sep2 >> type;

    if (in.fail() || sep1 != 'x' || sep2 != 'x')
    {
      throw gutil::IOException("Wrong format of RAW header information ("+std::string(name)+")");
    }

    msbfirst=false;

    if (!in.eof())
    {
      in >> order;

      if (order == 'm')
      {
        msbfirst=true;
      }
    }

    ret=s.substr(0, pos);
  }
  else
  {
    ret=s;
    s.replace(s.length()-4, 4, ".hdr");

    gutil::Properties prop(s.c_str());
    prop.getValue("image.width", width);
    prop.getValue("image.height", height);
    prop.getValue("image.pixelsize", type);
    prop.getValue("image.msbfirst", msbfirst, "false");
  }

  return ret;
}

void writeRAWHeader(const char *name, int type, long width, long height)
{
  gutil::Properties prop;

  prop.putValue("image.width", width);
  prop.putValue("image.height", height);
  prop.putValue("image.pixelsize", type);
  prop.putValue("image.msbfirst", false);

  std::string s=name;
  s.replace(s.length()-4, 4, ".hdr");

  prop.save(s.c_str(), (std::string("Header information of RAW File: ")+std::string(name)).c_str());
}

}

BasicImageIO *RAWImageIO::create() const
{
  return new RAWImageIO();
}

bool RAWImageIO::handlesFile(const char *name, bool reading) const
{
  std::string s=name;
  size_t pos;

  pos=s.find('&');

  if (pos != s.npos)
  {
    s=s.substr(0, pos);
  }

  if (s.size() <= 4)
  {
    return false;
  }

  if (s.find(".raw") == s.size()-4 || s.find(".RAW") == s.size()-4)
  {
    return true;
  }

  return false;
}

void RAWImageIO::loadHeader(const char *name, long &width, long &height,
                            int &depth) const
{
  int  type;
  bool msbfirst;

  if (!handlesFile(name, true))
  {
    throw gutil::IOException("Can only load RAW image ("+std::string(name)+")");
  }

  readRAWHeader(name, type, msbfirst, width, height);
  depth=1;
}

void RAWImageIO::load(ImageU8 &image, const char *name, int ds, long x, long y,
                      long w, long h) const
{
  std::string filename;
  long   width, height;
  int    type;
  bool   msbfirst;

  if (!handlesFile(name, true))
  {
    throw gutil::IOException("Can only load RAW image ("+std::string(name)+")");
  }

  filename=readRAWHeader(name, type, msbfirst, width, height);

  if (type > 1)
  {
    throw gutil::IOException("A 16 bit image cannot be loaded as 8 bit image ("+std::string(name)+")");
  }

  ds=std::max(1, ds);

  if (w < 0)
  {
    w=(width+ds-1)/ds;
  }

  if (h < 0)
  {
    h=(height+ds-1)/ds;
  }

  image.setSize(w, h, 1);
  image.clear();

  try
  {
    std::ifstream in;
    in.exceptions(std::ios_base::failbit | std::ios_base::badbit | std::ios_base::eofbit);
    in.open(filename.c_str(), std::ios::binary);

    // load downscaled part?

    if (ds > 1 || x != 0 || y != 0 || w != width || h != height)
    {
      std::valarray<ImageU8::work_t> vline(0, w);
      std::valarray<int> nline(0, w);

      for (long k=std::max(0l, -y); k<h && (y+k)*ds<height; k++)
      {
        // load downscaled line

        vline=0;
        nline=0;

        for (long kk=0; kk<ds && kk+(y+k)*ds<height; kk++)
        {
          in.seekg(static_cast<std::streamoff>((y+k)*ds+kk)*width+
                   static_cast<std::streamoff>(std::max(0l, x))*ds);

          std::streambuf *sb=in.rdbuf();

          long j=std::max(0l, -x);

          for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
          {
            for (int ii=0; ii<ds && (x+i)*ds+ii<width; ii++)
            {
              ImageU8::store_t v=static_cast<ImageU8::store_t>(sb->sbumpc());

              if (image.isValidS(v))
              {
                vline[j]+=v;
                nline[j]++;
              }
            }

            j++;
          }
        }

        // store line into image

        long j=std::max(0l, -x);

        for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
        {
          if (nline[j] > 0)
          {
            image.set(i, k, 0, static_cast<ImageU8::store_t>(vline[j]/nline[j]));
          }

          j++;
        }
      }
    }
    else // load whole image
    {
      std::streambuf *sb=in.rdbuf();

      for (long k=0; k<height; k++)
      {
        for (long i=0; i<width; i++)
        {
          image.set(i, k, 0, static_cast<ImageU8::store_t>(sb->sbumpc()));
        }
      }
    }

    in.close();
  }
  catch (std::ios_base::failure ex)
  {
    throw gutil::IOException(ex.what());
  }
}

void RAWImageIO::load(ImageU16 &image, const char *name, int ds, long x, long y,
                      long w, long h) const
{
  std::string filename;
  long   width, height;
  int    type;
  bool   msbfirst;

  if (!handlesFile(name, true))
  {
    throw gutil::IOException("Can only load RAW image ("+std::string(name)+")");
  }

  filename=readRAWHeader(name, type, msbfirst, width, height);

  if (type > 1)
  {
    ds=std::max(1, ds);

    if (w < 0)
    {
      w=(width+ds-1)/ds;
    }

    if (h < 0)
    {
      h=(height+ds-1)/ds;
    }

    image.setSize(w, h, 1);
    image.clear();

    try
    {
      std::ifstream in;
      in.exceptions(std::ios_base::failbit | std::ios_base::badbit | std::ios_base::eofbit);
      in.open(filename.c_str(), std::ios::binary);

      // load downscaled part?

      if (ds > 1 || x != 0 || y != 0 || w != width || h != height)
      {
        std::valarray<ImageU8::work_t> vline(0, w);
        std::valarray<int> nline(0, w);

        for (long k=std::max(0l, -y); k<h && (y+k)*ds<height; k++)
        {
          // load downscaled line

          vline=0;
          nline=0;

          for (long kk=0; kk<ds && kk+(y+k)*ds<height; kk++)
          {
            in.seekg(static_cast<std::streamoff>((y+k)*ds+kk)*width*2+
                     static_cast<std::streamoff>(std::max(0l, x))*ds*2);

            std::streambuf *sb=in.rdbuf();

            long j=std::max(0l, -x);

            for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
            {
              for (int ii=0; ii<ds && (x+i)*ds+ii<width; ii++)
              {
                ImageU16::store_t v;

                if (msbfirst)
                {
                  v=(static_cast<ImageU16::store_t>(sb->sbumpc())&0xff);
                  v=(v<<8)|(static_cast<ImageU16::store_t>(sb->sbumpc())&0xff);
                }
                else
                {
                  v=(static_cast<ImageU16::store_t>(sb->sbumpc())&0xff);
                  v=v|((static_cast<ImageU16::store_t>(sb->sbumpc())&0xff)<<8);
                }

                if (image.isValidS(v))
                {
                  vline[j]+=v;
                  nline[j]++;
                }
              }

              j++;
            }
          }

          // store line into image

          long j=std::max(0l, -x);

          for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
          {
            if (nline[j] > 0)
            {
              image.set(i, k, 0, static_cast<ImageU8::store_t>(vline[j]/nline[j]));
            }

            j++;
          }
        }
      }
      else // load whole image
      {
        std::streambuf *sb=in.rdbuf();

        for (long k=0; k<height; k++)
        {
          for (long i=0; i<width; i++)
          {
            ImageU16::store_t v;

            if (msbfirst)
            {
              v=(static_cast<ImageU16::store_t>(sb->sbumpc())&0xff);
              v=(v<<8)|(static_cast<ImageU16::store_t>(sb->sbumpc())&0xff);
            }
            else
            {
              v=(static_cast<ImageU16::store_t>(sb->sbumpc())&0xff);
              v=v|((static_cast<ImageU16::store_t>(sb->sbumpc())&0xff)<<8);
            }

            image.set(i, k, 0, v);
          }
        }
      }

      in.close();
    }
    catch (std::ios_base::failure ex)
    {
      throw gutil::IOException(ex.what());
    }
  }
  else
  {
    ImageU8 imageu8;
    load(imageu8, name, ds, x, y, w, h);
    image.setImageLimited(imageu8);
  }
}

void RAWImageIO::save(const ImageU8 &image, const char *name) const
{
  if (!handlesFile(name, false) || image.getDepth() != 1)
  {
    throw gutil::IOException("Can only save RAW images with depth 1 ("+std::string(name)+")");
  }

  writeRAWHeader(name, 1, image.getWidth(), image.getHeight());

  try
  {
    std::ofstream out;
    out.exceptions(std::ios_base::failbit | std::ios_base::badbit);
    out.open(name, std::ios::binary);

    std::streambuf *sb=out.rdbuf();

    for (long k=0; k<image.getHeight() && out.good(); k++)
    {
      for (long i=0; i<image.getWidth(); i++)
      {
        sb->sputc(static_cast<char>(image.get(i, k)));
      }
    }

    out.close();
  }
  catch (std::ios_base::failure ex)
  {
    throw gutil::IOException(ex.what());
  }
}

void RAWImageIO::save(const ImageU16 &image, const char *name) const
{
  if (!handlesFile(name, false) || image.getDepth() != 1)
  {
    throw gutil::IOException("Can only save RAW images with depth 1 ("+std::string(name)+")");
  }

  writeRAWHeader(name, 2, image.getWidth(), image.getHeight());

  try
  {
    std::ofstream out;
    out.exceptions(std::ios_base::failbit | std::ios_base::badbit);
    out.open(name, std::ios::binary);

    std::streambuf *sb=out.rdbuf();

    for (long k=0; k<image.getHeight() && out.good(); k++)
    {
      for (long i=0; i<image.getWidth(); i++)
      {
        ImageU16::store_t p=image.get(i, k);

        sb->sputc(static_cast<char>(p&0xff));
        sb->sputc(static_cast<char>((p>>8)&0xff));
      }
    }

    out.close();
  }
  catch (std::ios_base::failure ex)
  {
    throw gutil::IOException(ex.what());
  }
}

}