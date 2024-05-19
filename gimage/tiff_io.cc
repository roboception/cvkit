/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2023 Roboception GmbH, Munich, Germany
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

#include "tiff_io.h"

#include <gimage/image.h>
#include <gimage/size.h>

#include <stdexcept>
#include <vector>

#include <tiffio.h>

namespace gimage
{

BasicImageIO *TIFFImageIO::create() const
{
  return new TIFFImageIO();
}

bool TIFFImageIO::handlesFile(const char *name, bool reading) const
{
  std::string s=name;

  if (s.size() < 5)
  {
    return false;
  }

  if (s.rfind(".tif") == s.size()-4 || s.rfind(".TIF") == s.size()-4 ||
    s.rfind(".tiff") == s.size()-5 || s.rfind(".TIFF") == s.size()-5)
  {
    return true;
  }

  return false;
}

void TIFFImageIO::loadHeader(const char *name, long &width, long &height, int &depth) const
{
  TIFF *tif=TIFFOpen(name, "r");

  if (tif == 0)
  {
    throw gutil::IOException("Can only load TIFF image ("+std::string(name)+")");
  }

  uint32_t w, h;
  uint16_t d;

  TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w);
  TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h);
  TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &d);

  width=w;
  height=h;
  depth=d;

  TIFFClose(tif);
}

namespace
{

template<class T> class TiffImage
{
  public:

    TIFF *tif;

    uint32_t width;
    uint32_t height;
    uint16_t depth;       // samples per pixel
    uint16_t bits;        // bits per sample
    bool is_float;        // integer or float
    bool is_planar;       // multiple planes all samples in row
    size_t raw_size;      // size of raw row
    uint16_t photometric; // photometric interpretation
                          // 0: WhiteIsZero, 1: BlackIsZero, 2: RGB, Other: Use ReadRGBA

    gimage::Image<T> rows; // temporary part of the image
    std::vector<T> row; // temporary row of unpacked values (planar or all samples in row)
    std::vector<uint8_t> raw; // temporary row of raw, packed values

    TiffImage(const char *name)
    {
      tif=TIFFOpen(name, "r");

      if (tif)
      {
        TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
        TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
        TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &depth);
        TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bits);

        uint16_t format;
        uint16_t planar;

        TIFFGetField(tif, TIFFTAG_SAMPLEFORMAT, &format);
        TIFFGetField(tif, TIFFTAG_PLANARCONFIG, &planar);

        is_float=(format == SAMPLEFORMAT_IEEEFP);
        is_planar=(planar == PLANARCONFIG_SEPARATE);

        raw_size=TIFFScanlineSize(tif);
        TIFFGetField(tif, TIFFTAG_PHOTOMETRIC, &photometric);
      }
      else
      {
        throw gutil::IOException("Can only load TIFF image ("+std::string(name)+")");
      }
    }

    ~TiffImage()
    {
      TIFFClose(tif);
    }
};

/*
  Reading of a TIFF scanline, unpacking and casting to the data type of the
  line parameter.

  NOTE: The function may use the temporary array tif.raw.

  @param ret     Returned row of unpacked values
  @param n       Size of array ret
  @param tif     Tiff image data.
  @param row     Row to be loaded
*/

template<class T> void readUnpacked(T *ret, uint32_t n, TiffImage<T> &tif, uint32_t row, uint16_t sample)
{
  if (tif.bits != 8*sizeof(T) || tif.is_float != std::is_floating_point<T>::value ||
    n*sizeof(T) < tif.raw_size || tif.photometric == PHOTOMETRIC_MINISWHITE)
  {
    // converting row value wise

    tif.raw.resize(tif.raw_size);

    if (TIFFReadScanline(tif.tif, tif.raw.data(), row, sample) < 0)
    {
      throw gutil::IOException("Cannot read row "+std::to_string(row)+" of TIFF image");
    }

    if (tif.is_float)
    {
      if (tif.bits == 32)
      {
        float *p=reinterpret_cast<float *>(tif.raw.data());

        if (tif.photometric == PHOTOMETRIC_MINISWHITE)
        {
          for (uint32_t i=0; i<n; i++) ret[i]=static_cast<T>(1.0f-p[i]);
        }
        else
        {
          for (uint32_t i=0; i<n; i++) ret[i]=static_cast<T>(p[i]);
        }
      }
      else if (tif.bits == 64)
      {
        double *p=reinterpret_cast<double *>(tif.raw.data());

        if (tif.photometric == PHOTOMETRIC_MINISWHITE)
        {
          for (uint32_t i=0; i<n; i++) ret[i]=static_cast<T>(1.0-p[i]);
        }
        else
        {
          for (uint32_t i=0; i<n; i++) ret[i]=static_cast<T>(p[i]);
        }
      }
      else
      {
        throw gutil::IOException("Cannot convert TIFF image with "+std::to_string(tif.bits)+" bit floating point type");
      }
    }
    else
    {
      if (tif.bits == 1)
      {
        uint8_t *p=reinterpret_cast<uint8_t *>(tif.raw.data());

        size_t i=0;
        if (tif.photometric == PHOTOMETRIC_MINISWHITE)
        {
          while (i < n)
          {
            ret[i++]=static_cast<T>(1-((*p>>7) & 0x1));
            if (i < n) ret[i++]=static_cast<T>(1-((*p>>6) & 0x1));
            if (i < n) ret[i++]=static_cast<T>(1-((*p>>5) & 0x1));
            if (i < n) ret[i++]=static_cast<T>(1-((*p>>4) & 0x1));
            if (i < n) ret[i++]=static_cast<T>(1-((*p>>3) & 0x1));
            if (i < n) ret[i++]=static_cast<T>(1-((*p>>2) & 0x1));
            if (i < n) ret[i++]=static_cast<T>(1-((*p>>1) & 0x1));
            if (i < n) ret[i++]=static_cast<T>(1-(*p & 0x1));
            p++;
          }
        }
        else
        {
          while (i < n)
          {
            ret[i++]=static_cast<T>((*p>>7) & 0x1);
            if (i < n) ret[i++]=static_cast<T>((*p>>6) & 0x1);
            if (i < n) ret[i++]=static_cast<T>((*p>>5) & 0x1);
            if (i < n) ret[i++]=static_cast<T>((*p>>4) & 0x1);
            if (i < n) ret[i++]=static_cast<T>((*p>>3) & 0x1);
            if (i < n) ret[i++]=static_cast<T>((*p>>2) & 0x1);
            if (i < n) ret[i++]=static_cast<T>((*p>>1) & 0x1);
            if (i < n) ret[i++]=static_cast<T>(*p & 0x1);
            p++;
          }
        }
      }
      else if (tif.bits == 2)
      {
        uint8_t *p=reinterpret_cast<uint8_t *>(tif.raw.data());

        size_t i=0;
        if (tif.photometric == PHOTOMETRIC_MINISWHITE)
        {
          while (i < n)
          {
            ret[i++]=static_cast<T>(3-((*p>>6) & 0x3));
            if (i < n) ret[i++]=static_cast<T>(3-((*p>>4) & 0x3));
            if (i < n) ret[i++]=static_cast<T>(3-((*p>>2) & 0x3));
            if (i < n) ret[i++]=static_cast<T>(3-(*p & 0x3));
            p++;
          }
        }
        else
        {
          while (i < n)
          {
            ret[i++]=static_cast<T>((*p>>6) & 0x3);
            if (i < n) ret[i++]=static_cast<T>((*p>>4) & 0x3);
            if (i < n) ret[i++]=static_cast<T>((*p>>2) & 0x3);
            if (i < n) ret[i++]=static_cast<T>(*p & 0x3);
            p++;
          }
        }
      }
      else if (tif.bits == 4)
      {
        uint8_t *p=reinterpret_cast<uint8_t *>(tif.raw.data());

        size_t i=0;
        if (tif.photometric == PHOTOMETRIC_MINISWHITE)
        {
          while (i < n)
          {
            ret[i++]=static_cast<T>(15-((*p>>4) & 0xf));
            if (i < n) ret[i++]=static_cast<T>(15-(*p & 0xf));
            p++;
          }
        }
        else
        {
          while (i < n)
          {
            ret[i++]=static_cast<T>((*p>>4) & 0xf);
            if (i < n) ret[i++]=static_cast<T>(*p & 0xf);
            p++;
          }
        }
      }
      else if (tif.bits == 8)
      {
        uint8_t *p=reinterpret_cast<uint8_t *>(tif.raw.data());
        if (tif.photometric == PHOTOMETRIC_MINISWHITE)
        {
          for (size_t i=0; i<n; i++) ret[i]=static_cast<T>(255-p[i]);
        }
        else
        {
          for (size_t i=0; i<n; i++) ret[i]=static_cast<T>(p[i]);
        }
      }
      else if (tif.bits == 16)
      {
        uint16_t *p=reinterpret_cast<uint16_t *>(tif.raw.data());
        if (tif.photometric == PHOTOMETRIC_MINISWHITE)
        {
          for (size_t i=0; i<n; i++) ret[i]=static_cast<T>(65535-p[i]);
        }
        else
        {
          for (size_t i=0; i<n; i++) ret[i]=static_cast<T>(p[i]);
        }
      }
      else if (tif.bits == 32)
      {
        uint32_t *p=reinterpret_cast<uint32_t *>(tif.raw.data());
        if (tif.photometric == PHOTOMETRIC_MINISWHITE)
        {
          for (size_t i=0; i<n; i++) ret[i]=static_cast<T>(4294967295-p[i]);
        }
        else
        {
          for (size_t i=0; i<n; i++) ret[i]=static_cast<T>(p[i]);
        }
      }
      else
      {
        throw gutil::IOException("Cannot convert TIFF image with "+std::to_string(tif.bits)+" bit integer type");
      }
    }
  }
  else
  {
    if (TIFFReadScanline(tif.tif, ret, row, sample) < 0)
    {
      throw gutil::IOException("Cannot read row "+std::to_string(row)+" of TIFF image");
    }
  }
}

/*
  Reading of a row into the provided image.

  NOTE: The function may use the temporary arrays tif.row and tif.raw.

  @param ret Image into which a row will be loaded.
  @param k   Target row in image ret.
  @param d   Target depth in image (only used for planar configuration)
  @param tif Tiff image data.
  @param row Row to be loaded
*/

template<class T> void readRow(gimage::Image<T> &ret, uint32_t k, TiffImage<T> &tif, uint32_t row,
  uint16_t d)
{
  ret.setSize(tif.width, tif.height, tif.depth);

  if (tif.is_planar || tif.depth == 1)
  {
    readUnpacked(ret.getPtr(0, k, d), tif.width, tif, row, d);
  }
  else
  {
    tif.row.resize(tif.width*tif.depth);
    readUnpacked(tif.row.data(), static_cast<uint32_t>(tif.row.size()), tif, row, 0);

    T *p=tif.row.data();
    for (uint32_t i=0; i<tif.width; i++)
    {
      for (uint16_t d=0; d<tif.depth; d++)
      {
        ret.set(i, k, d, *p++);
      }
    }
  }
}

template<class T> void loadInternal(Image<T> &image, const char *name, int ds, long x, long y,
  long w, long h)
{
  TiffImage<T> tif(name);

  if (tif.bits > 8*sizeof(T))
  {
    throw gutil::IOException("Image "+std::string(name)+" has "+
      std::to_string(tif.bits)+" bits per sample, but target image only "+std::to_string(8*sizeof(T)));
  }

  if (w < 0)
  {
    w=(tif.width+ds-1)/ds;
  }

  if (h < 0)
  {
    h=(tif.height+ds-1)/ds;
  }

  if (tif.photometric == PHOTOMETRIC_MINISWHITE || tif.photometric == PHOTOMETRIC_MINISBLACK ||
    tif.photometric == PHOTOMETRIC_RGB)
  {
    // loading full image (could be optimised with downscaling and cropping
    // while loading row-wise)

    image.setSize(tif.width, tif.height, tif.depth);

    if (tif.is_planar)
    {
      for (uint16_t d=0; d<tif.depth; d++)
      {
        for (long k=0; k<h; k++)
        {
          readRow(image, static_cast<uint32_t>(k), tif, static_cast<uint32_t>(k), d);
        }
      }
    }
    else
    {
      for (long k=0; k<h; k++)
      {
        readRow(image, static_cast<uint32_t>(k), tif, static_cast<uint32_t>(k), 0);
      }
    }

    // optionally downscale

    if (ds > 1)
    {
      image=gimage::downscaleImage(image, ds);
    }

    // optionally copy part

    if (x != 0 || y != 0 || w != image.getWidth() || h != image.getHeight())
    {
      image=gimage::cropImage(image, x, y, w, h);
    }
  }
  else
  {
    // load full image

    {
      std::vector<uint32_t> pixel(tif.width*tif.height);
      if (TIFFReadRGBAImageOriented(tif.tif, tif.width, tif.height, pixel.data(),
        ORIENTATION_TOPLEFT, 1) <= 0)
      {
        throw gutil::IOException("Cannot load image "+std::string(name));
      }

      // convert into RGB image

      image.setSize(tif.width, tif.height, 3);

      uint32_t *p=pixel.data();
      for (long k=0; k<image.getHeight(); k++)
      {
        for (long i=0; i<image.getWidth(); i++)
        {
          image.set(i, k, 0, static_cast<T>(TIFFGetR(*p)));
          image.set(i, k, 1, static_cast<T>(TIFFGetG(*p)));
          image.set(i, k, 2, static_cast<T>(TIFFGetB(*p)));
          p++;
        }
      }
    }

    // optionally downscale

    if (ds > 1)
    {
      image=gimage::downscaleImage(image, ds);
    }

    // optionally copy part

    if (x != 0 || y != 0 || w != image.getWidth() || h != image.getHeight())
    {
      image=gimage::cropImage(image, x, y, w, h);
    }
  }
}

}

void TIFFImageIO::load(ImageU8 &image, const char *name, int ds, long x, long y, long w, long h) const
{
  loadInternal(image, name, ds, x, y, w, h);
}

void TIFFImageIO::load(ImageU16 &image, const char *name, int ds, long x, long y, long w, long h) const
{
  loadInternal(image, name, ds, x, y, w, h);
}

void TIFFImageIO::load(ImageFloat &image, const char *name, int ds, long x, long y, long w, long h) const
{
  loadInternal(image, name, ds, x, y, w, h);
}

namespace
{

template<class T> void saveInternal(const Image<T> &image, const char *name)
{
  TIFF* tif=TIFFOpen(name, "w");

  if (!tif)
  {
    throw gutil::IOException("Cannot store image: "+std::string(name));
  }

  TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, static_cast<uint32_t>(image.getWidth()));
  TIFFSetField(tif, TIFFTAG_IMAGELENGTH, static_cast<uint32_t>(image.getHeight()));
  TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, static_cast<uint16_t>(image.getDepth()));
  TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8*sizeof(T));

  if (std::is_floating_point<T>::value)
  {
    TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
  }
  else
  {
    TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_UINT);
  }

  int strip=static_cast<int>(8192/(image.getWidth()*image.getDepth()*sizeof(T)));
  strip=std::max(1, strip);
  strip=std::min(static_cast<int>(image.getHeight()), strip);

  TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tif, static_cast<uint32_t>(strip)));

  TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
  TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);

  if (image.getDepth() == 3 || image.getDepth() == 4)
  {
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
  }
  else
  {
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
  }

  if (image.getDepth() == 1)
  {
    for (long k=0; k<image.getHeight(); k++)
    {
      if (TIFFWriteScanline(tif, image.getPtr(0, k, 0), static_cast<uint32_t>(k), 0) < 0)
      {
        TIFFClose(tif);
        throw gutil::IOException("Cannot write image data: "+std::string(name));
      }
    }
  }
  else
  {
    std::vector<T> line(image.getWidth()*image.getDepth());

    for (long k=0; k<image.getHeight(); k++)
    {
      T *p=line.data();
      for (long i=0; i<image.getWidth(); i++)
      {
        for (int d=0; d<image.getDepth(); d++)
        {
          *p++=image.get(i, k, d);
        }
      }

      if (TIFFWriteScanline(tif, line.data(), static_cast<uint32_t>(k), 0) < 0)
      {
        TIFFClose(tif);
        throw gutil::IOException("Cannot write image data: "+std::string(name));
      }
    }
  }

  TIFFClose(tif);
}

}

void TIFFImageIO::save(const ImageU8 &image, const char *name) const
{
  saveInternal(image, name);
}

void TIFFImageIO::save(const ImageU16 &image, const char *name) const
{
  saveInternal(image, name);
}

void TIFFImageIO::save(const ImageFloat &image, const char *name) const
{
  saveInternal(image, name);
}

}
