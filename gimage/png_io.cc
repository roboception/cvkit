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

#include "png_io.h"

#include <gutil/version.h>

#include <stdexcept>
#include <sstream>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <valarray>

#include <png.h>
#include <time.h>

namespace gimage
{

BasicImageIO *PNGImageIO::create() const
{
  return new PNGImageIO();
}

bool PNGImageIO::handlesFile(const char *name, bool reading) const
{
  std::string s=name;

  if (s.size() <= 4)
  {
    return false;
  }

  if (s.rfind(".png") == s.size()-4 || s.rfind(".PNG") == s.size()-4)
  {
    return true;
  }

  return false;
}

void PNGImageIO::loadHeader(const char *name, long &width, long &height,
                            int &depth) const
{
  if (!handlesFile(name, true))
  {
    throw gutil::IOException("Can only load PNG image ("+std::string(name)+")");
  }

  // initialize reading a png file

  FILE *in=fopen(name, "rb");

  if (in == 0)
  {
    throw gutil::IOException("Cannot open file ("+std::string(name)+")");
  }

  png_structp png=png_create_read_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);

  if (png == 0)
  {
    fclose(in);
    throw gutil::IOException("Cannot allocate hangle for reading PNG files");
  }

  png_infop info=png_create_info_struct(png);

  if (info == 0)
  {
    png_destroy_read_struct(&png, static_cast<png_infopp>(0), static_cast<png_infopp>(0));
    fclose(in);
    throw gutil::IOException("Cannot allocate PNG info structure");
  }

  png_infop end=png_create_info_struct(png);

  if (end == 0)
  {
    png_destroy_read_struct(&png, &info, static_cast<png_infopp>(0));
    fclose(in);
    throw gutil::IOException("Cannot allocate PNG info structure");
  }

  if (setjmp(png_jmpbuf(png)))
  {
    png_destroy_read_struct(&png, &info, &end);
    fclose(in);
    throw gutil::IOException("Cannot read PNG file ("+std::string(name)+")");
  }

  // read header data

  png_init_io(png, in);
  png_read_info(png, info);

  width=static_cast<long>(png_get_image_width(png, info));
  height=static_cast<long>(png_get_image_height(png, info));

  depth=1;
  int color=png_get_color_type(png, info);

  if (color == PNG_COLOR_TYPE_PALETTE || color == PNG_COLOR_TYPE_RGB ||
      color == PNG_COLOR_TYPE_RGB_ALPHA)
  {
    depth=3;
  }

  // close data set

  png_destroy_read_struct(&png, &info, &end);
  fclose(in);
}

void PNGImageIO::load(ImageU8 &image, const char *name, int ds, long x, long y,
                      long w, long h) const
{
  if (!handlesFile(name, true))
  {
    throw gutil::IOException("Can only load PNG image ("+std::string(name)+")");
  }

  // initialize reading a png file

  FILE *in=fopen(name, "rb");

  if (in == 0)
  {
    throw gutil::IOException("Cannot open file ("+std::string(name)+")");
  }

  png_structp png=png_create_read_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);

  if (png == 0)
  {
    fclose(in);
    throw gutil::IOException("Cannot allocate hangle for reading PNG files");
  }

  png_infop info=png_create_info_struct(png);

  if (info == 0)
  {
    png_destroy_read_struct(&png, static_cast<png_infopp>(0), static_cast<png_infopp>(0));
    fclose(in);
    throw gutil::IOException("Cannot allocate PNG info structure");
  }

  png_infop end=png_create_info_struct(png);

  if (end == 0)
  {
    png_destroy_read_struct(&png, &info, static_cast<png_infopp>(0));
    fclose(in);
    throw gutil::IOException("Cannot allocate PNG info structure");
  }

  unsigned char *img=0;
  unsigned char **row=0;

  if (setjmp(png_jmpbuf(png)))
  {
    delete [] img;
    delete [] row;

    png_destroy_read_struct(&png, &info, &end);
    fclose(in);
    throw gutil::IOException("Cannot read PNG file ("+std::string(name)+")");
  }

  // read header data

  png_init_io(png, in);
  png_read_info(png, info);

  long width=static_cast<long>(png_get_image_width(png, info));
  long height=static_cast<long>(png_get_image_height(png, info));

  int depth=1;
  int color=png_get_color_type(png, info);

  if (color == PNG_COLOR_TYPE_PALETTE || color == PNG_COLOR_TYPE_RGB ||
      color == PNG_COLOR_TYPE_RGB_ALPHA)
  {
    depth=3;
  }

  // adapt size of image

  if (w < 0)
  {
    w=(width+ds-1)/ds;
  }

  if (h < 0)
  {
    h=(height+ds-1)/ds;
  }

  image.setSize(w, h, depth);
  image.clear();

  // transform palette images to RGB

  if (color == PNG_COLOR_TYPE_PALETTE)
  {
    png_set_palette_to_rgb(png);
  }

  // make sure that there are 8 Bits per color channel

  int bits=png_get_bit_depth(png, info);

  if (bits > 8)
  {
    png_destroy_read_struct(&png, &info, &end);
    fclose(in);
    throw gutil::IOException("PNG with <= 8 bits expected ("+std::string(name)+")");
  }

  if (color == PNG_COLOR_TYPE_GRAY && bits < 8)
  {
    png_set_expand_gray_1_2_4_to_8(png);
  }

  // strip alpha channel if available

  if ((color & PNG_COLOR_MASK_ALPHA) != 0)
  {
    png_set_strip_alpha(png);
  }

  png_read_update_info(png, info);

  // read image completely

  int rn=png_get_rowbytes(png, info);
  img=new unsigned char [height*rn];
  row=new unsigned char * [height];

  for (int k=0; k<height; k++)
  {
    row[k]=img+k*rn;
  }

  png_read_image(png, static_cast<png_bytepp>(row));

  // load downscaled part?

  if (ds > 1 || x != 0 || y != 0 || w != width || h != height)
  {
    std::valarray<ImageU8::work_t> vline(0, w*depth);
    std::valarray<int> nline(0, w*depth);

    // skip the first y*ds image rows

    int rk=std::max(0l, y*ds);

    for (long k=std::max(0l, -y); k<h && (y+k)*ds<height; k++)
    {
      // load downscaled line

      vline=0;
      nline=0;

      for (long kk=0; kk<ds && kk+(y+k)*ds<height; kk++)
      {
        int  jj=std::max(0l, x)*ds*depth;
        long j=std::max(0l, -x)*depth;

        for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
        {
          for (int ii=0; ii<ds && (x+i)*ds+ii<width; ii++)
          {
            for (int d=0; d<depth; d++)
            {
              ImageU8::store_t v=static_cast<ImageU8::store_t>(row[rk][jj++]);

              if (image.isValidS(v))
              {
                vline[j+d]+=v;
                nline[j+d]++;
              }
            }
          }

          j+=depth;
        }

        rk++;
      }

      // store line into image

      long j=std::max(0l, -x)*depth;

      for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
      {
        for (int d=0; d<depth; d++)
        {
          if (nline[j] > 0)
          {
            image.set(i, k, d, static_cast<ImageU8::store_t>((vline[j]+nline[j]/2)/nline[j]));
          }

          j++;
        }
      }
    }
  }
  else // load whole image
  {
    for (long k=0; k<height; k++)
    {
      int j=0;

      for (long i=0; i<width; i++)
      {
        for (int d=0; d<depth; d++)
        {
          image.set(i, k, d, static_cast<ImageU8::store_t>(row[k][j++]));
        }
      }
    }
  }

  // close data set

  delete [] row;
  delete [] img;

  png_destroy_read_struct(&png, &info, &end);
  fclose(in);
}

void PNGImageIO::load(ImageU16 &image, const char *name, int ds, long x, long y,
                      long w, long h) const
{
  if (!handlesFile(name, true))
  {
    throw gutil::IOException("Can only load PNG image ("+std::string(name)+")");
  }

  // initialize reading a png file

  FILE *in=fopen(name, "rb");

  if (in == 0)
  {
    throw gutil::IOException("Cannot open file ("+std::string(name)+")");
  }

  png_structp png=png_create_read_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);

  if (png == 0)
  {
    fclose(in);
    throw gutil::IOException("Cannot allocate hangle for reading PNG files");
  }

  png_infop info=png_create_info_struct(png);

  if (info == 0)
  {
    png_destroy_read_struct(&png, static_cast<png_infopp>(0), static_cast<png_infopp>(0));
    fclose(in);
    throw gutil::IOException("Cannot allocate PNG info structure");
  }

  png_infop end=png_create_info_struct(png);

  if (end == 0)
  {
    png_destroy_read_struct(&png, &info, static_cast<png_infopp>(0));
    fclose(in);
    throw gutil::IOException("Cannot allocate PNG info structure");
  }

  unsigned char *img=0;
  unsigned char **row=0;

  if (setjmp(png_jmpbuf(png)))
  {
    delete [] img;
    delete [] row;

    png_destroy_read_struct(&png, &info, &end);
    fclose(in);
    throw gutil::IOException("Cannot read PNG file ("+std::string(name)+")");
  }

  // read header data

  png_init_io(png, in);
  png_read_info(png, info);

  long width=static_cast<long>(png_get_image_width(png, info));
  long height=static_cast<long>(png_get_image_height(png, info));

  int depth=1;
  int color=png_get_color_type(png, info);

  if (color == PNG_COLOR_TYPE_PALETTE || color == PNG_COLOR_TYPE_RGB ||
      color == PNG_COLOR_TYPE_RGB_ALPHA)
  {
    depth=3;
  }

  // adapt size of image

  if (w < 0)
  {
    w=(width+ds-1)/ds;
  }

  if (h < 0)
  {
    h=(height+ds-1)/ds;
  }

  image.setSize(w, h, depth);
  image.clear();

  // transform palette images to RGB

  if (color == PNG_COLOR_TYPE_PALETTE)
  {
    png_set_palette_to_rgb(png);
  }

  int bits=png_get_bit_depth(png, info);

  if (color == PNG_COLOR_TYPE_GRAY && bits < 8)
  {
    png_set_expand_gray_1_2_4_to_8(png);
  }

  // strip alpha channel if available

  if ((color & PNG_COLOR_MASK_ALPHA) != 0)
  {
    png_set_strip_alpha(png);
  }

  // read image completely

  int rn=png_get_rowbytes(png, info);
  img=new unsigned char [height*rn];
  row=new unsigned char * [height];

  for (int k=0; k<height; k++)
  {
    row[k]=img+k*rn;
  }

  png_read_image(png, static_cast<png_bytepp>(row));

  // load downscaled part?

  if (ds > 1 || x != 0 || y != 0 || w != width || h != height)
  {
    std::valarray<ImageU16::work_t> vline(0, w*depth);
    std::valarray<int> nline(0, w*depth);

    // skip the first y*ds image rows

    int rk=std::max(0l, y*ds);

    for (long k=std::max(0l, -y); k<h && (y+k)*ds<height; k++)
    {
      // load downscaled line

      vline=0;
      nline=0;

      for (long kk=0; kk<ds && kk+(y+k)*ds<height; kk++)
      {
        int  jj=std::max(0l, x)*ds*depth;
        long j=std::max(0l, -x)*depth;

        for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
        {
          for (int ii=0; ii<ds && (x+i)*ds+ii<width; ii++)
          {
            for (int d=0; d<depth; d++)
            {
              ImageU16::store_t v;

              if (bits < 16)
              {
                v=static_cast<ImageU16::store_t>(row[rk][jj]);
                jj++;
              }
              else
              {
                v=static_cast<ImageU16::store_t>(row[rk][jj]<<8) | row[rk][jj+1];
                jj+=2;
              }

              if (image.isValidS(v))
              {
                vline[j+d]+=v;
                nline[j+d]++;
              }
            }
          }

          j+=depth;
        }

        rk++;
      }

      // store line into image

      long j=std::max(0l, -x)*depth;

      for (long i=std::max(0l, -x); i<w && (x+i)*ds<width; i++)
      {
        for (int d=0; d<depth; d++)
        {
          if (nline[j] > 0)
          {
            image.set(i, k, d, static_cast<ImageU8::store_t>((vline[j]+nline[j]/2)/nline[j]));
          }

          j++;
        }
      }
    }
  }
  else // load whole image
  {
    for (long k=0; k<height; k++)
    {
      int j=0;

      for (long i=0; i<width; i++)
      {
        for (int d=0; d<depth; d++)
        {
          ImageU16::store_t v;

          if (bits < 16)
          {
            v=static_cast<ImageU16::store_t>(row[k][j]);
            j++;
          }
          else
          {
            v=static_cast<ImageU16::store_t>(row[k][j]<<8) | row[k][j+1];
            j+=2;
          }

          image.set(i, k, d, v);
        }
      }
    }
  }

  // close data set

  delete [] row;
  delete [] img;

  png_destroy_read_struct(&png, &info, &end);
  fclose(in);
}

void PNGImageIO::save(const ImageU8 &image, const char *name) const
{
  if (!handlesFile(name, false) || (image.getDepth() != 1 && image.getDepth() != 3))
  {
    throw gutil::IOException("Can only save PNG images with depth 1 or 3 ("+std::string(name)+")");
  }

  // initialize writing a png file

  FILE *out=fopen(name, "wb");

  if (out == 0)
  {
    throw gutil::IOException("Cannot open file ("+std::string(name)+")");
  }

  png_structp png=png_create_write_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);

  if (png == 0)
  {
    fclose(out);
    throw gutil::IOException("Cannot allocate hangle for writing PNG files");
  }

  png_infop info=png_create_info_struct(png);

  if (info == 0)
  {
    png_destroy_write_struct(&png, static_cast<png_infopp>(0));
    fclose(out);
    throw gutil::IOException("Cannot allocate PNG info structure");
  }

  unsigned char *row=0;

  if (setjmp(png_jmpbuf(png)))
  {
    delete [] row;

    png_destroy_write_struct(&png, &info);
    fclose(out);
    throw gutil::IOException("Cannot read PNG file ("+std::string(name)+")");
  }

  // write header

  png_init_io(png, out);

  int color=PNG_COLOR_TYPE_GRAY;

  if (image.getDepth() == 3)
  {
    color=PNG_COLOR_TYPE_RGB;
  }

  png_set_IHDR(png, info, image.getWidth(), image.getHeight(), 8, color,
               PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
               PNG_FILTER_TYPE_DEFAULT);

  std::string vs=std::string("cvkit version ")+std::string(VERSION);

  time_t tt=time(0);
  std::string tm=ctime(&tt);

  png_text text[2];
  memset(text, 0, 2*sizeof(png_text));

  text[0].compression=PNG_TEXT_COMPRESSION_NONE;
  text[0].key=const_cast<char *>("Software");
  text[0].text=const_cast<char *>(vs.c_str());
  text[0].text_length=strlen(text[0].text);

  text[1].compression=PNG_TEXT_COMPRESSION_NONE;
  text[1].key=const_cast<char *>("Creation Time");
  text[1].text=const_cast<char *>(tm.c_str());
  text[1].text_length=strlen(text[1].text);

  png_set_text(png, info, text, 2);

  png_write_info(png, info);

  // write image

  row=new unsigned char [image.getDepth()*image.getWidth()];

  // write image content line by line

  for (long k=0; k<image.getHeight(); k++)
  {
    unsigned char *rp=row;

    for (long i=0; i<image.getWidth(); i++)
    {
      for (int j=0; j<image.getDepth(); j++)
      {
        *rp++=image.get(i, k, j);
      }
    }

    png_write_row(png, row);
  }

  // finish writing and close file

  png_write_end(png, info);

  delete [] row;

  fclose(out);

  png_destroy_write_struct(&png, &info);
}

void PNGImageIO::save(const ImageU16 &image, const char *name) const
{
  if (!handlesFile(name, false) || (image.getDepth() != 1 && image.getDepth() != 3))
  {
    throw gutil::IOException("Can only save PNG images with depth 1 or 3 ("+std::string(name)+")");
  }

  // initialize writing a png file

  FILE *out=fopen(name, "wb");

  if (out == 0)
  {
    throw gutil::IOException("Cannot open file ("+std::string(name)+")");
  }

  png_structp png=png_create_write_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);

  if (png == 0)
  {
    fclose(out);
    throw gutil::IOException("Cannot allocate hangle for writing PNG files");
  }

  png_infop info=png_create_info_struct(png);

  if (info == 0)
  {
    png_destroy_write_struct(&png, static_cast<png_infopp>(0));
    fclose(out);
    throw gutil::IOException("Cannot allocate PNG info structure");
  }

  unsigned char *row=0;

  if (setjmp(png_jmpbuf(png)))
  {
    delete [] row;

    png_destroy_write_struct(&png, &info);
    fclose(out);
    throw gutil::IOException("Cannot read PNG file ("+std::string(name)+")");
  }

  // write header

  png_init_io(png, out);

  int color=PNG_COLOR_TYPE_GRAY;

  if (image.getDepth() == 3)
  {
    color=PNG_COLOR_TYPE_RGB;
  }

  png_set_IHDR(png, info, image.getWidth(), image.getHeight(), 16, color,
               PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
               PNG_FILTER_TYPE_DEFAULT);

  std::string vs=std::string("cvkit version ")+std::string(VERSION);

  time_t tt=time(0);
  std::string tm=ctime(&tt);

  png_text text[2];
  memset(text, 0, 2*sizeof(png_text));

  text[0].compression=PNG_TEXT_COMPRESSION_NONE;
  text[0].key=const_cast<char *>("Software");
  text[0].text=const_cast<char *>(vs.c_str());
  text[0].text_length=strlen(text[0].text);

  text[1].compression=PNG_TEXT_COMPRESSION_NONE;
  text[1].key=const_cast<char *>("Creation Time");
  text[1].text=const_cast<char *>(tm.c_str());
  text[1].text_length=strlen(text[1].text);

  png_set_text(png, info, text, 2);

  png_write_info(png, info);

  // write image

  row=new unsigned char [2*image.getDepth()*image.getWidth()];

  // write image content line by line

  for (long k=0; k<image.getHeight(); k++)
  {
    unsigned char *rp=row;

    for (long i=0; i<image.getWidth(); i++)
    {
      for (int j=0; j<image.getDepth(); j++)
      {
        *rp++=static_cast<unsigned char>(image.get(i, k, j)>>8);
        *rp++=static_cast<unsigned char>(image.get(i, k, j)&0xff);
      }
    }

    png_write_row(png, row);
  }

  // finish writing and close file

  png_write_end(png, info);

  delete [] row;

  fclose(out);

  png_destroy_write_struct(&png, &info);
}

}
