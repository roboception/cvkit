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

#include "jpeg_io.h"

#include <stdexcept>
#include <sstream>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <valarray>

#include <jpeglib.h>

using std::string;
using std::valarray;
using std::max;

using gutil::IOException;

namespace gimage
{

BasicImageIO *JPEGImageIO::create() const
{
    return new JPEGImageIO();
}

bool JPEGImageIO::handlesFile(const char *name, bool reading) const
{
    string s=name;
    
    if (s.size() <= 4)
      return false;
    
    if (s.rfind(".jpg") == s.size()-4 || s.rfind(".JPG") == s.size()-4)
      return true;
    
    return false;
}

void JPEGImageIO::loadHeader(const char *name, long &width, long &height,
  int &depth) const
{
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE   *in=0;
    
    if (!handlesFile(name, true))
      throw IOException("Can only load JPG image ("+string(name)+")");
    
      // open output file using C methods
    
    in=fopen(name, "rb");
    if (in == 0)
      throw IOException("Cannot open file for reading ("+string(name)+")");
    
      // installing standard error handler, create compression object and set output file
    
    cinfo.err=jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, in);
    
      // read header information
    
    jpeg_read_header(&cinfo, TRUE);
    
    width=static_cast<long>(cinfo.image_width);
    height=static_cast<long>(cinfo.image_height);
    depth=cinfo.num_components;
    
      // close object and input stream
    
    jpeg_destroy_decompress(&cinfo);
    fclose(in);
}

void JPEGImageIO::load(ImageU8 &image, const char *name, int ds, long x, long y,
  long w, long h) const
{
    long  width, height;
    int   depth;
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row=0;
    FILE   *in=0;
    
    if (!handlesFile(name, true))
      throw IOException("Can only load JPG image ("+string(name)+")");
    
      // open output file using C methods
    
    in=fopen(name, "rb");
    if (in == 0)
      throw IOException("Cannot open file for reading ("+string(name)+")");
    
      // installing standard error handler, create compression object and set output file
    
    cinfo.err=jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, in);
    
      // read header information
    
    jpeg_read_header(&cinfo, TRUE);
    
      // use scaling possibility of JPEG library for increasing speed
    
    ds=max(1, ds);
    
    if (ds > 1)
    {
      if (ds%8 == 0)
      {
        cinfo.scale_denom=8;
        ds/=8;
      }
      else if (ds%4 == 0)
      {
        cinfo.scale_denom=4;
        ds/=4;
      }
      else if (ds%2 == 0)
      {
        cinfo.scale_denom=2;
        ds/=2;
      }
    }
    
      // start decompression
    
    jpeg_start_decompress(&cinfo);
    
      // get size and depth
    
    width=static_cast<long>(cinfo.output_width);
    height=static_cast<long>(cinfo.output_height);
    depth=cinfo.output_components;
    
    if (w < 0)
      w=(width+ds-1)/ds;
    
    if (h < 0)
      h=(height+ds-1)/ds;
    
    image.setSize(w, h, depth);
    image.clear();
    
      // allocate image row buffer
    
    row=new JSAMPLE [width*depth];
    
      // load downscaled part?
    
    if (ds > 1 || x != 0 || y != 0 || w != width || h != height)
    {
      valarray<ImageU8::work_t> vline(0, w*depth);
      valarray<int> nline(0, w*depth);
      
        // skip the first y*ds image rows
      
      for (long k=0; k<y*ds && k<height; k++)
        jpeg_read_scanlines(&cinfo, &row, 1);
      
      for (long k=max(0l, -y); k<h && (y+k)*ds<height; k++)
      {
          // load downscaled line
        
        vline=0;
        nline=0;
        
        for (long kk=0; kk<ds && kk+(y+k)*ds<height; kk++)
        {
          jpeg_read_scanlines(&cinfo, &row, 1);
          
          int  jj=max(0l, x)*ds*depth;
          long j=max(0l, -x)*depth;
          for (long i=max(0l, -x); i<w && (x+i)*ds<width; i++)
          {
            for (int ii=0; ii<ds && (x+i)*ds+ii<width; ii++)
            {
              for (int d=0; d<depth; d++)
              {
                ImageU8::store_t v=static_cast<ImageU8::store_t>(row[jj++]);
                
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
        
        long j=max(0l, -x)*depth;
        for (long i=max(0l, -x); i<w && (x+i)*ds<width; i++)
        {
          for (int d=0; d<depth; d++)
          {
            if (nline[j] > 0)
              image.set(i, k, d, static_cast<ImageU8::store_t>(vline[j]/nline[j]));
            
            j++;
          }
        }
      }
      
      if (cinfo.output_scanline < cinfo.output_height)
        jpeg_abort_decompress(&cinfo);
      else
        jpeg_finish_decompress(&cinfo);
    }
    else // load whole image
    {
      for (long k=0; k<height; k++)
      {
        jpeg_read_scanlines(&cinfo, &row, 1);
        
        int j=0;
        for (long i=0; i<width; i++)
        {
          for (int d=0; d<depth; d++)
            image.set(i, k, d, static_cast<ImageU8::store_t>(row[j++]));
        }
      }
      
      jpeg_finish_decompress(&cinfo);
    }
    
      // close object and input stream
    
    jpeg_destroy_decompress(&cinfo);
    fclose(in);
    delete [] row;
}

void JPEGImageIO::save(const ImageU8 &image, const char *name) const
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row=0;
    FILE   *out=0;
    
    if (!handlesFile(name, false) || (image.getDepth() != 1 && image.getDepth() != 3))
      throw IOException("Can only save JPG images with depth 1 or 3 ("+string(name)+")");
    
      // open output file using C methods
    
    out=fopen(name, "wb");
    if (out == 0)
      throw IOException("Cannot open file for writing ("+string(name)+")");
    
      // installing standard error handler, create compression object and set output file
    
    cinfo.err=jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, out);
    
      // set image size, color space and quality
    
    cinfo.image_width=static_cast<JDIMENSION>(image.getWidth());
    cinfo.image_height=static_cast<JDIMENSION>(image.getHeight());
    cinfo.input_components=image.getDepth();
    
    cinfo.in_color_space=JCS_GRAYSCALE;
    if (image.getDepth() == 3)
      cinfo.in_color_space=JCS_RGB;
    
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 90, TRUE);
    
      // start compression
    
    jpeg_start_compress(&cinfo, TRUE);
    row=new JSAMPLE [image.getWidth()*image.getDepth()];
    
      // write image content line by line
    
    for (long k=0; k<image.getHeight(); k++)
    {
      JSAMPROW rp=row;
      for (long i=0; i<image.getWidth(); i++)
      {
        for (int j=0; j<image.getDepth(); j++)
          *rp++=static_cast<JSAMPLE>(image.get(i, k, j));
      }
      
      jpeg_write_scanlines(&cinfo, &row, 1);
    }
    
      // finish compression and close stream
    
    jpeg_finish_compress(&cinfo);
    delete [] row;
    
    fclose(out);
    
    jpeg_destroy_compress(&cinfo);
}

}
