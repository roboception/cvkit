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

#include "gdal_io.h"

#include <gutil/misc.h>

#include <gdal.h>
#include <cpl_string.h>
#include <ios>
#include <valarray>
#include <set>

namespace gimage
{

namespace
{

/**
 * Custom GDAL error handler that suppresses warnings.
 */

void customGDALErrorHandler(CPLErr err, int n, const char *msg)
{
  if (err == CE_Failure || err == CE_Fatal)
  {
    CPLDefaultErrorHandler(err, n, msg);
  }
}

/**
 * Returns the maximum value that must be available for holding the given GDAL
 * data type.
 */

double getMaxValue(GDALDataType type)
{
  double ret;

  switch (type)
  {
    case GDT_Byte:
      ret=255;
      break;

    case GDT_UInt16:
      ret=65535;
      break;

    case GDT_Int16:
    case GDT_UInt32:
    case GDT_Int32:
    case GDT_Float32:
      ret=std::numeric_limits<float>::max();
      break;

    default:
      ret=0;
      break;
  }

  return ret;
}

/**
 * Determines the number of bands and the maximum value that can hold the data
 * without saturation.
 */

int getMaxBand(GDALDatasetH gd, double &mm)
{
  int ret=0;

  mm=0;

  int n=GDALGetRasterCount(gd);

  for (int i=0; i<n; i++)
  {
    double m=getMaxValue(GDALGetRasterDataType(GDALGetRasterBand(gd, i+1)));

    if (m > 0)
    {
      mm=std::max(m, mm);
      ret++;
    }
  }

  return ret;
}

}

GDALImageIO::GDALImageIO()
{
  GDALAllRegister();
  CPLSetErrorHandler(customGDALErrorHandler);
}

GDALImageIO::~GDALImageIO()
{
  GDALDestroyDriverManager();
}

BasicImageIO *GDALImageIO::create() const
{
  return new GDALImageIO();
}

bool GDALImageIO::handlesFile(const char *name, bool reading) const
{
  std::string s=name;

  if (reading)
  {
    // check if name refers to a tiled image

    std::set<std::string> list;
    size_t pos=s.rfind(':');

    if (pos != s.npos && s.compare(pos, 2, ":\\") == 0)
    {
      pos=s.npos;
    }

    if (pos != s.npos)
    {
      std::string prefix=s.substr(0, pos);
      std::string suffix=s.substr(pos+1);

      // get name of one tile

      gutil::getFileList(list, prefix, suffix);

      if (list.size() > 0)
      {
        name=list.begin()->c_str();
      }
    }

    // use function from GDAL to check if there is an available driver

    if (GDALIdentifyDriver(name, 0) != 0)
    {
      return true;
    }
  }
  else if (s.size() > 4 && (s.rfind(".tif") == s.size()-4 ||
                            s.rfind(".TIF") == s.size()-4))
  {
    GDALDriverH gd=GDALGetDriverByName("GTiff");
    char **meta=GDALGetMetadata(gd, 0);

    if (CSLFetchBoolean(meta, GDAL_DCAP_CREATE, false))
    {
      return true;
    }
  }

  return false;
}

void GDALImageIO::loadHeader(const char *name, long &width, long &height,
                             int &depth) const
{
  GDALDatasetH gd=GDALOpen(name, GA_ReadOnly);
  double       m;

  if (gd == 0)
  {
    throw gutil::IOException("File is not supported by GDAL ("+std::string(name)+")");
  }

  width=GDALGetRasterXSize(gd);
  height=GDALGetRasterYSize(gd);
  depth=getMaxBand(gd, m);

  GDALClose(gd);
}

void GDALImageIO::loadProperties(gutil::Properties &prop, const char *name) const
{
  // read tags

  GDALDatasetH gd=GDALOpen(name, GA_ReadOnly);

  if (gd != 0)
  {
    char **meta=GDALGetMetadata(gd, "");

    if (meta != 0)
    {
      while (*meta != 0)
      {
        std::string s=*meta;
        size_t pos=s.find('=');

        if (pos != s.npos)
        {
          prop.putString(s.substr(0, pos).c_str(), s.substr(pos+1));
        }

        meta++;
      }
    }

    GDALClose(gd);
  }

  // read and translate orientation from optional EXIF tags

  int v;
  prop.getValue("EXIF_Orientation", v, "0");

  switch (v)
  {
    case 1:
      prop.putString("rotation", "0");
      prop.putString("flip", "0");
      break;

    case 2:
      prop.putString("rotation", "0");
      prop.putString("flip", "1");
      break;

    case 3:
      prop.putString("rotation", "180");
      prop.putString("flip", "0");
      break;

    case 4:
      prop.putString("rotation", "180");
      prop.putString("flip", "1");
      break;

    case 5:
      prop.putString("rotation", "90");
      prop.putString("flip", "1");
      break;

    case 6:
      prop.putString("rotation", "90");
      prop.putString("flip", "0");
      break;

    case 7:
      prop.putString("rotation", "270");
      prop.putString("flip", "1");
      break;

    case 8:
      prop.putString("rotation", "270");
      prop.putString("flip", "0");
      break;

    default:
      break;
  }

  // read and translate approximate focal length from optional EXIF tags

  double f;
  int    w;
  int    h;

  prop.getValue("EXIF_FocalLengthIn35mmFilm", f, "0");
  prop.getValue("EXIF_PixelXDimension", w, "0");
  prop.getValue("EXIF_PixelYDimension", h, "0");

  if (f > 0 && w > 0 && h > 0)
  {
    prop.putValue("f", f/36*w);
  }

  BasicImageIO::loadProperties(prop, name);
}

namespace
{

int convertColorTable(GDALColorTableH gct, std::valarray<short> &rgba)
{
  int n=GDALGetColorEntryCount(gct);
  GDALColorEntry v;

  rgba.resize(4*n, 0);

  for (int i=0; i<n; i++)
  {
    GDALGetColorEntryAsRGB(gct, i, &v);
    rgba[(i<<2)]=v.c1;
    rgba[(i<<2)+1]=v.c2;
    rgba[(i<<2)+2]=v.c3;
    rgba[(i<<2)+3]=v.c4;
  }

  return n;
}

template<class T> void loadInternal(Image<T> &image, GDALDataType gt,
                                    const char *name, int ds, long x, long y, long w, long h)
{
  GDALDatasetH gd=GDALOpen(name, GA_ReadOnly);

  if (gd == 0)
  {
    throw gutil::IOException("File is not supported by GDAL ("+std::string(name)+")");
  }

  try
  {
    double m;
    int depth=getMaxBand(gd, m);
    int rgba_count=-1;
    std::valarray<short> rgba;

    // check for and read color table

    if (depth == 1)
    {
      GDALRasterBandH gb=GDALGetRasterBand(gd, 1);
      GDALColorTableH gct=0;

      gct=GDALGetRasterColorTable(gb);

      if (gct != 0)
      {
        depth=3;
        rgba_count=convertColorTable(gct, rgba);
        m=rgba.max();
      }
    }

    // check data size

    if (m == 0 || m > image.absMaxValue())
    {
      std::ostringstream s;
      s << "The image data type is too big: " << m << ">" << image.absMaxValue() << " (" << name << ")";
      throw gutil::IOException(s.str());
    }

    // determine size in file

    long xf=0;
    long yf=0;
    long wf=GDALGetRasterXSize(gd);
    long hf=GDALGetRasterYSize(gd);

    if (w < 0)
    {
      w=(wf-x*ds+ds-1)/ds;
    }

    if (h < 0)
    {
      h=(hf-y*ds+ds-1)/ds;
    }

    // set image size

    image.setSize(w, h, depth);
    image.clear();

    // adapt image buffer part and image file part to each other

    if (x < 0)
    {
      x=-x;
      w-=x;
    }
    else
    {
      xf=x*ds;
      wf-=xf;
      x=0;
    }

    if (y < 0)
    {
      y=-y;
      h-=y;
    }
    else
    {
      yf=y*ds;
      hf-=yf;
      y=0;
    }

    if (w*ds > wf)
    {
      w=(wf+ds-1)/ds;
    }
    else
    {
      wf=w*ds;
    }

    if (h*ds > hf)
    {
      h=(hf+ds-1)/ds;
    }
    else
    {
      hf=h*ds;
    }

    // loading bands directly or via color table

    if (rgba_count < 0)
    {
      T *p=new T [w*h];
      int d=0;
      int n=GDALGetRasterCount(gd);

      for (int j=0; j<n; j++)
      {
        GDALRasterBandH gb=GDALGetRasterBand(gd, j+1);

        if (getMaxValue(GDALGetRasterDataType(gb)) > 0)
        {
          if (GDALRasterIO(gb, GF_Read, xf, yf, wf, hf, p, w, h, gt,
                           sizeof(T), w*sizeof(T)) < CE_Failure)
          {
            for (int k=0; k<h; k++)
            {
              for (int i=0; i<w; i++)
              {
                image.set(x+i, y+k, d, p[k*w+i]);
              }
            }
          }
          else
          {
            delete [] p;

            std::ostringstream s;
            s << "Cannot read raster band " << j << " of image (" << name << ")";
            throw gutil::IOException(s.str());
          }

          d++;
        }
      }

      delete [] p;
    }
    else
    {
      short *p=new short [w*h];

      GDALRasterBandH gb=GDALGetRasterBand(gd, 1);

      if (GDALRasterIO(gb, GF_Read, xf, yf, wf, hf, p, w, h, GDT_Int16,
                       sizeof(short), w*sizeof(short)) < CE_Failure)
      {
        for (int k=0; k<h; k++)
        {
          for (int i=0; i<w; i++)
          {
            int j=p[k*w+i];

            if (j >= 0 && j < rgba_count)
            {
              j<<=2;
              image.set(x+i, y+k, 0, static_cast<T>(rgba[j++]));
              image.set(x+i, y+k, 1, static_cast<T>(rgba[j++]));
              image.set(x+i, y+k, 2, static_cast<T>(rgba[j++]));
            }
          }
        }
      }
      else
      {
        delete [] p;

        std::ostringstream s;
        s << "Cannot read raster band 1 of image (" << name << ") with color table";
        throw gutil::IOException(s.str());
      }

      delete [] p;
    }
  }
  catch (...)
  {
    GDALClose(gd);
    throw;
  }

  GDALClose(gd);
}

}

void GDALImageIO::load(ImageU8 &image, const char *name, int ds, long x, long y,
                       long w, long h) const
{
  loadInternal(image, GDT_Byte, name, ds, x, y, w, h);
}

void GDALImageIO::load(ImageU16 &image, const char *name, int ds, long x,
                       long y, long w, long h) const
{
  loadInternal(image, GDT_UInt16, name, ds, x, y, w, h);
}

void GDALImageIO::load(ImageFloat &image, const char *name, int ds, long x,
                       long y, long w, long h) const
{
  loadInternal(image, GDT_Float32, name, ds, x, y, w, h);
}

namespace
{

template <class T> void saveInternal(const Image<T> &image, const char *name,
                                     GDALDataType gt)
{
  std::string s=name;
  GDALDriverH gdriver=0;

  // find driver according to suffix

  if (s.rfind(".tif") == s.size()-4 || s.rfind(".TIF") == s.size()-4)
  {
    gdriver=GDALGetDriverByName("GTiff");
  }

  if (gdriver == 0)
  {
    throw gutil::IOException("Saving is only supported as TIF ("+std::string(name)+")");
  }

  // create new data set

  char *gp[2];

  gp[0]=0;

  if (image.getDepth() >= 3)
  {
    gp[0]=(char *) "PHOTOMETRIC=RGB";
  }

  gp[1]=0;

  GDALDatasetH gd=GDALCreate(gdriver, name, image.getWidth(),
                             image.getHeight(), image.getDepth(), gt, gp);

  if (gd == 0)
  {
    throw gutil::IOException("Cannot save image ("+std::string(name)+")");
  }

  try
  {
    // store image

    T *p=new T [image.getWidth()*image.getHeight()];

    for (int d=0; d<image.getDepth(); d++)
    {
      GDALRasterBandH gb=GDALGetRasterBand(gd, d+1);

      for (int k=0; k<image.getHeight(); k++)
      {
        for (int i=0; i<image.getWidth(); i++)
        {
          p[k*image.getWidth()+i]=image.get(i, k, d);
        }
      }

      if (GDALRasterIO(gb, GF_Write, 0, 0, image.getWidth(),
                       image.getHeight(), p, image.getWidth(), image.getHeight(), gt,
                       sizeof(T), image.getWidth()*sizeof(T)) >= CE_Failure)
      {
        delete [] p;

        throw gutil::IOException("Cannot save image ("+std::string(name)+")");
      }
    }

    delete [] p;
  }
  catch (...)
  {
    GDALClose(gd);
    throw;
  }

  GDALClose(gd);
}

}

void GDALImageIO::save(const ImageU8 &image, const char *name) const
{
  saveInternal(image, name, GDT_Byte);
}

void GDALImageIO::save(const ImageU16 &image, const char *name) const
{
  saveInternal(image, name, GDT_UInt16);
}

void GDALImageIO::save(const ImageFloat &image, const char *name) const
{
  saveInternal(image, name, GDT_Float32);
}

}
