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

#include "pnm_io.h"

#include "arithmetic.h"

#include <gutil/misc.h>

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

std::string readPNMToken(std::istream &in)
{
    char c;
    std::ostringstream s;

      // read the first character of the token (ignoring white spaces and
      // comments in front the token)

    in.get(c);
    while (!in.eof())
    {
      while (!in.eof() && isspace(c))
        in.get(c);

      if (c == '#')
      {
        while (!in.eof() && c != '\n' && c != '\r')
          in.get(c);
      }
      else
        break;
    }

      // reading a token until white space or start of comment

    while (!in.eof() && !isspace(c) && c != '#')
    {
      s << c;
      in.get(c);
    }

      // put last character back if it was the start of a comment

    if (!in.eof() && c == '#')
      in.unget();

    return s.str();
}

std::istream::pos_type readPNMHeader(const char *name, int &ncomp, long &maxval,
  float &scale, long &width, long &height)
{
    std::string            s;
    std::istream::pos_type ret=0;

    try
    {
      std::ifstream in;
      in.exceptions(std::ios_base::failbit | std::ios_base::badbit | std::ios_base::eofbit);
      in.open(name, std::ios::binary);

      s=readPNMToken(in);

      if (s == "P5" || s == "Pf")
        ncomp=1;
      else if (s == "P6" || s == "PF")
        ncomp=3;
      else
        throw gutil::IOException("Unknown PNM image type '"+s+"' of image "+name);

      width=atol(readPNMToken(in).c_str());
      height=atol(readPNMToken(in).c_str());

      maxval=0;
      scale=0;
      if (s == "Pf" || s == "PF")
        scale=atof(readPNMToken(in).c_str());
      else
        maxval=atol(readPNMToken(in).c_str());

      if (width == 0 || height == 0 || (maxval == 0 && scale== 0))
      {
        std::ostringstream ss;
        ss << "Invalid PNM image (" << width << " " << height << " " << maxval;
        ss << " " << scale << ") of image " << name;
        throw gutil::IOException(ss.str());
      }

      ret=in.tellg();
      in.close();
    }
    catch (std::ios_base::failure ex)
    {
      throw gutil::IOException(ex.what());
    }

    return ret;
}

void writePNMHeader(const char *name, const char *type, long width,
  long height, long maxval, float scale)
{
    try
    {
      std::ofstream out;
      out.exceptions(std::ios_base::failbit | std::ios_base::badbit);

      out.open(name, std::ios::binary);
      out << type << std::endl;
      out << width << " " << height << std::endl;

      if (strcmp(type, "Pf") == 0 || strcmp(type, "PF") == 0)
        out << scale << "\n";
      else
        out << maxval << "\n";

      out.close();
    }
    catch (std::ios_base::failure ex)
    {
      throw gutil::IOException(ex.what());
    }
}

}

BasicImageIO *PNMImageIO::create() const
{
    return new PNMImageIO();
}

bool PNMImageIO::handlesFile(const char *name, bool reading) const
{
    std::string s=name;

    if (s.size() <= 4)
      return false;

    if (s.rfind(".pgm") == s.size()-4 || s.rfind(".PGM") == s.size()-4 ||
      s.rfind(".ppm") == s.size()-4 || s.rfind(".PPM") == s.size()-4 ||
      s.rfind(".pfm") == s.size()-4 || s.rfind(".PFM") == s.size()-4)
      return true;

    return false;
}

void PNMImageIO::loadHeader(const char *name, long &width, long &height,
  int &depth) const
{
    long  maxval;
    float scale;

    if (!handlesFile(name, true))
      throw gutil::IOException("Can only load PNM image ("+std::string(name)+")");

    readPNMHeader(name, depth, maxval, scale, width, height);
}

void PNMImageIO::load(ImageU8 &image, const char *name, int ds, long x, long y,
  long w, long h) const
{
    long  width, height, maxval;
    float scale;
    int   depth;
    std::istream::pos_type pos;

    if (!handlesFile(name, true))
      throw gutil::IOException("Can only load PNM image ("+std::string(name)+")");

    pos=readPNMHeader(name, depth, maxval, scale, width, height);

    if (scale != 0)
      throw gutil::IOException("A float image cannot be loaded as 8 bit image ("+std::string(name)+")");

    if (maxval > 255)
      throw gutil::IOException("A 16 bit image cannot be loaded as 8 bit image ("+std::string(name)+")");

    ds=std::max(1, ds);

    if (w < 0)
      w=(width+ds-1)/ds;

    if (h < 0)
      h=(height+ds-1)/ds;

    image.setSize(w, h, depth);
    image.clear();

    try
    {
      std::ifstream in;
      in.exceptions(std::ios_base::failbit | std::ios_base::badbit | std::ios_base::eofbit);
      in.open(name, std::ios::binary);

        // load downscaled part?

      if (ds > 1 || x != 0 || y != 0 || w != width || h != height)
      {
        std::valarray<ImageU8::work_t> vline(0, w*depth);
        std::valarray<int> nline(0, w*depth);

        for (long k=std::max(0l, -y); k<h && (y+k)*ds<height; k++)
        {
            // load downscaled line

          vline=0;
          nline=0;

          for (long kk=0; kk<ds && kk+(y+k)*ds<height; kk++)
          {
            in.seekg(pos+static_cast<std::streamoff>((y+k)*ds+kk)*width*depth+
              static_cast<std::streamoff>(std::max(0l, x))*ds*depth);

            std::streambuf *sb=in.rdbuf();

            long j=std::max(0l, -x)*depth;
            for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
            {
              for (int ii=0; ii<ds && (x+i)*ds+ii<width; ii++)
              {
                for (int d=0; d<depth; d++)
                {
                  ImageU8::store_t v=static_cast<ImageU8::store_t>(sb->sbumpc());

                  if (image.isValidS(v))
                  {
                    vline[j+d]+=v;
                    nline[j+d]++;
                  }
                }
              }

              j+=depth;
            }
          }

            // store line into image

          long j=std::max(0l, -x)*depth;
          for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
          {
            for (int d=0; d<depth; d++)
            {
              if (nline[j] > 0)
                image.set(i, k, d, static_cast<ImageU8::store_t>(vline[j]/nline[j]));

              j++;
            }
          }
        }
      }
      else // load whole image
      {
        in.seekg(pos);
        std::streambuf *sb=in.rdbuf();

        for (long k=0; k<height; k++)
        {
          for (long i=0; i<width; i++)
          {
            for (int d=0; d<depth; d++)
              image.set(i, k, d, static_cast<ImageU8::store_t>(sb->sbumpc()));
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

void PNMImageIO::load(ImageU16 &image, const char *name, int ds, long x,
  long y, long w, long h) const
{
    long  width, height, maxval;
    float scale;
    int   depth;
    std::istream::pos_type pos;

    if (!handlesFile(name, true))
      throw gutil::IOException("Can only load PNM image ("+std::string(name)+")");

    pos=readPNMHeader(name, depth, maxval, scale, width, height);

    if (scale != 0)
      throw gutil::IOException("A float image cannot be loaded as 16 bit image ("+std::string(name)+")");

    if (maxval > 255)
    {
      ds=std::max(1, ds);

      if (w < 0)
        w=(width+ds-1)/ds;

      if (h < 0)
        h=(height+ds-1)/ds;

      image.setSize(w, h, depth);
      image.clear();

      try
      {
        std::ifstream in;
        in.exceptions(std::ios_base::failbit | std::ios_base::badbit | std::ios_base::eofbit);
        in.open(name, std::ios::binary);

          // load downscaled part?

        if (ds > 1 || x != 0 || y != 0 || w != width || h != height)
        {
          std::valarray<ImageU16::work_t> vline(0, w*depth);
          std::valarray<int> nline(0, w*depth);

          for (long k=std::max(0l, -y); k<h && (y+k)*ds<height; k++)
          {
              // load downscaled line

            vline=0;
            nline=0;

            for (long kk=0; kk<ds && kk+(y+k)*ds<height; kk++)
            {
              in.seekg(pos+static_cast<std::streamoff>((y+k)*ds+kk)*width*depth*2+
                static_cast<std::streamoff>(std::max(0l, x))*ds*depth*2);

              std::streambuf *sb=in.rdbuf();

              long j=std::max(0l, -x)*depth;
              for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
              {
                for (int ii=0; ii<ds && (x+i)*ds+ii<width; ii++)
                {
                  for (int d=0; d<depth; d++)
                  {
                    ImageU16::store_t v;

                    v=(static_cast<ImageU16::store_t>(sb->sbumpc())&0xff);
                    v=(v<<8)|(static_cast<ImageU16::store_t>(sb->sbumpc())&0xff);

                    if (image.isValidS(v))
                    {
                      vline[j+d]+=v;
                      nline[j+d]++;
                    }
                  }
                }

                j+=depth;
              }
            }

              // store line into image

            long j=std::max(0l, -x)*depth;
            for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
            {
              for (int d=0; d<depth; d++)
              {
                if (nline[j] > 0)
                  image.set(i, k, d, static_cast<ImageU16::store_t>(vline[j]/nline[j]));

                j++;
              }
            }
          }
        }
        else // load whole image
        {
          in.seekg(pos);
          std::streambuf *sb=in.rdbuf();

          for (long k=0; k<height; k++)
          {
            for (long i=0; i<width; i++)
            {
              for (int d=0; d<depth; d++)
              {
                ImageU16::store_t v;

                v=(static_cast<ImageU16::store_t>(sb->sbumpc())&0xff);
                v=(v<<8)|(static_cast<ImageU16::store_t>(sb->sbumpc())&0xff);

                image.set(i, k, d, v);
              }
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

void PNMImageIO::load(ImageFloat &image, const char *name, int ds, long x,
  long y, long w, long h) const
{
    long  width, height, maxval;
    float scale;
    int   depth;
    std::istream::pos_type pos;
    float p;
    char *c=reinterpret_cast<char *>(&p);
    bool msbfirst=gutil::isMSBFirst();

    if (!handlesFile(name, true))
      throw gutil::IOException("Can only load PNM image ("+std::string(name)+")");

    pos=readPNMHeader(name, depth, maxval, scale, width, height);

    if (scale != 0)
    {
      ds=std::max(1, ds);

      if (w < 0)
        w=(width+ds-1)/ds;

      if (h < 0)
        h=(height+ds-1)/ds;

      image.setSize(w, h, depth);
      image.clear();

      try
      {
        std::ifstream in;
        in.exceptions(std::ios_base::failbit | std::ios_base::badbit | std::ios_base::eofbit);
        in.open(name, std::ios::binary);

          // load downscaled part?

        if (ds > 1 || x != 0 || y != 0 || w != width || h != height)
        {
          std::valarray<float> vline(0.0f, w*depth);
          std::valarray<int> nline(0, w*depth);

          for (long k=std::max(0l, -y); k<h && (y+k)*ds<height; k++)
          {
              // load downscaled line

            vline=0;
            nline=0;

            for (long kk=0; kk<ds && kk+(y+k)*ds<height; kk++)
            {
              in.seekg(pos+static_cast<std::streamoff>(height-1-(y+k)*ds-kk)*width*depth*4+
                static_cast<std::streamoff>(std::max(0l, x))*ds*depth*4);

              std::streambuf *sb=in.rdbuf();

              long j=std::max(0l, -x)*depth;
              for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
              {
                for (int ii=0; ii<ds && (x+i)*ds+ii<width; ii++)
                {
                  for (int d=0; d<depth; d++)
                  {
                      // we assume that the plattform uses IEEE 32 bit floating
                      // point format, otherwise this will not work

                    if ((scale > 0 && msbfirst) || (scale < 0 && !msbfirst))
                    {
                      c[0]=sb->sbumpc();
                      c[1]=sb->sbumpc();
                      c[2]=sb->sbumpc();
                      c[3]=sb->sbumpc();
                    }
                    else
                    {
                      c[3]=sb->sbumpc();
                      c[2]=sb->sbumpc();
                      c[1]=sb->sbumpc();
                      c[0]=sb->sbumpc();
                    }

                    if (image.isValidS(p))
                    {
                      vline[j+d]+=p;
                      nline[j+d]++;
                    }
                  }
                }

                j+=depth;
              }
            }

              // store line into image

            long j=std::max(0l, -x)*depth;
            for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
            {
              for (int d=0; d<depth; d++)
              {
                if (nline[j] > 0)
                  image.set(i, k, d, vline[j]/nline[j]);

                j++;
              }
            }
          }
        }
        else // load whole image
        {
          in.seekg(pos);
          std::streambuf *sb=in.rdbuf();

          for (long k=height-1; k>=0; k--)
          {
            for (long i=0; i<width; i++)
            {
              for (int d=0; d<depth; d++)
              {
                  // we assume that the plattform uses IEEE 32 bit floating
                  // point format, otherwise this will not work

                if ((scale > 0 && msbfirst) || (scale < 0 && !msbfirst))
                {
                  c[0]=sb->sbumpc();
                  c[1]=sb->sbumpc();
                  c[2]=sb->sbumpc();
                  c[3]=sb->sbumpc();
                }
                else
                {
                  c[3]=sb->sbumpc();
                  c[2]=sb->sbumpc();
                  c[1]=sb->sbumpc();
                  c[0]=sb->sbumpc();
                }

                image.set(i, k, d, p);
              }
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
    else if (maxval > 255)
    {
      ImageU16 imageu16;
      load(imageu16, name, ds, x, y, w, h);
      image.setImageLimited(imageu16);

        // support for reading legacy disparity images in fixed point format

      std::string s=name;
      if (s.rfind("_disp.pgm") == s.size()-9)
        s=s.substr(0, s.rfind("_disp.pgm"));
      else if (s.rfind("_height.pgm") == s.size()-11)
        s=s.substr(0, s.rfind("_height.pgm"));
      else
        s="";

      if (s.size() > 0)
      {
        gutil::Properties prop;

        try
        {
          size_t pos=s.rfind('_');

          if (pos != s.npos)
            pos=s.rfind('_', pos-1);

          if (pos != s.npos)
            prop.load((s.substr(0, pos)+"_param.txt").c_str());
        }
        catch (std::exception)
        { }

        try
        {
          prop.load((s+"_param.txt").c_str());
        }
        catch (std::exception)
        { }

        if (prop.contains("subbit"))
        {
            // invalid disparities in legacy disparity images have a value > 32767

          for (long k=0; k<image.getHeight(); k++)
          {
            for (long i=0; i<image.getWidth(); i++)
            {
              if (image.get(i, k) > 32767)
                image.setInvalid(i, k, 0);
            }
          }

            // consider sub-pixel setting

          int subbit;
          prop.getValue("subbit", subbit);

          if (subbit > 0)
            image/=1<<subbit;
        }

          // consider offset

        if (prop.contains("origin.d"))
        {
          double offset;
          prop.getValue("origin.d", offset);

          if (offset != 0)
            image+=offset;
        }
      }
    }
    else
    {
      ImageU8 imageu8;
      load(imageu8, name, ds, x, y, w, h);
      image.setImageLimited(imageu8);
    }
}

void PNMImageIO::save(const ImageU8 &image, const char *name) const
{
    if (!handlesFile(name, false) || (image.getDepth() != 1 && image.getDepth() != 3))
      throw gutil::IOException("Can only save PNM images with depth 1 or 3 ("+std::string(name)+")");

    std::string type="P5";
    if (image.getDepth() == 3)
      type="P6";

    writePNMHeader(name, type.c_str(), image.getWidth(), image.getHeight(),
      255, 0);

    try
    {
      std::ofstream out;
      out.exceptions(std::ios_base::failbit | std::ios_base::badbit);
      out.open(name, std::ios::binary|std::ios::app);

      std::streambuf *sb=out.rdbuf();

      for (long k=0; k<image.getHeight() && out.good(); k++)
      {
        for (long i=0; i<image.getWidth(); i++)
        {
          for (int j=0; j<image.getDepth(); j++)
            sb->sputc(static_cast<char>(image.get(i, k, j)));
        }
      }

      out.close();
    }
    catch (std::ios_base::failure ex)
    {
      throw gutil::IOException(ex.what());
    }
}

void PNMImageIO::save(const ImageU16 &image, const char *name) const
{
    if (!handlesFile(name, false) || (image.getDepth() != 1 && image.getDepth() != 3))
      throw gutil::IOException("Can only save PNM images with depth 1 or 3 ("+std::string(name)+")");

    std::string type="P5";
    if (image.getDepth() == 3)
      type="P6";

    writePNMHeader(name, type.c_str(), image.getWidth(), image.getHeight(),
      65535, 0);

    try
    {
      std::ofstream out;
      out.exceptions(std::ios_base::failbit | std::ios_base::badbit);
      out.open(name, std::ios::binary|std::ios::app);

      std::streambuf *sb=out.rdbuf();

      for (long k=0; k<image.getHeight() && out.good(); k++)
      {
        for (long i=0; i<image.getWidth(); i++)
        {
          for (int j=0; j<image.getDepth(); j++)
          {
            ImageU16::store_t p=image.get(i, k, j);

            sb->sputc(static_cast<char>((p>>8)&0xff));
            sb->sputc(static_cast<char>(p&0xff));
          }
        }
      }

      out.close();
    }
    catch (std::ios_base::failure ex)
    {
      throw gutil::IOException(ex.what());
    }
}

void PNMImageIO::save(const ImageFloat &image, const char *name) const
{
    float p, s;
    char *c=reinterpret_cast<char *>(&p);
    bool msbfirst=gutil::isMSBFirst();

    if (!handlesFile(name, false) || (image.getDepth() != 1 && image.getDepth() != 3))
      throw gutil::IOException("Can only save PNM images with depth 1 or 3 ("+std::string(name)+")");

    std::string type="Pf";
    if (image.getDepth() == 3)
      type="PF";

    s=image.maxValue();

    if (s > 0)
      s=1/s;
    else
      s=1;

    writePNMHeader(name, type.c_str(), image.getWidth(), image.getHeight(),
      0, s);

    try
    {
      std::ofstream out;
      out.exceptions(std::ios_base::failbit | std::ios_base::badbit);
      out.open(name, std::ios::binary|std::ios::app);

      std::streambuf *sb=out.rdbuf();

      for (long k=image.getHeight()-1; k>=0 && out.good(); k--)
      {
        for (long i=0; i<image.getWidth(); i++)
        {
          for (int j=0; j<image.getDepth(); j++)
          {
              // we assume that the plattform uses IEEE 32 bit floating
              // point format, otherwise this will not work

            p=static_cast<float>(image.get(i, k, j));

            if (msbfirst)
            {
              sb->sputc(c[0]);
              sb->sputc(c[1]);
              sb->sputc(c[2]);
              sb->sputc(c[3]);
            }
            else
            {
              sb->sputc(c[3]);
              sb->sputc(c[2]);
              sb->sputc(c[1]);
              sb->sputc(c[0]);
            }
          }
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