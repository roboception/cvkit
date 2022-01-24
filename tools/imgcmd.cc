/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2021, Roboception GmbH
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

#include <gimage/image.h>
#include <gimage/io.h>
#include <gimage/color.h>
#include <gimage/size.h>
#include <gimage/size.h>
#include <gimage/gauss.h>
#include <gimage/gauss_pyramid.h>
#include <gimage/hdr.h>
#include <gimage/noise.h>
#include <gimage/analysis.h>
#include <gimage/arithmetic.h>
#include <gimage/paint.h>
#include <gimage/compare.h>

#include <gutil/parameter.h>
#include <gutil/misc.h>
#include <gutil/proctime.h>
#include <gutil/version.h>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>

namespace
{

std::string nextParameterFilename(gutil::Parameter &param, const std::string &repl)
{
  std::string prefix;
  std::string suffix;

  param.nextString(prefix);

  size_t pos=prefix.find('%');

  if (pos < prefix.size())
  {
    suffix=prefix.substr(pos+1);
    prefix=prefix.substr(0, pos);
  }

  return prefix+repl+suffix;
}

template<class T> void process(gimage::Image<T> &image, gutil::Parameter param,
                               const std::string &repl)
{
  try
  {
    while (param.remaining() > 0)
    {
      std::string p;

      param.nextParameter(p);

      if (p == "-out")
      {
        gimage::getImageIO().save(image, nextParameterFilename(param, repl).c_str());
      }

      if (p == "-ds")
      {
        int factor;

        param.nextValue(factor);
        image=downscaleImage(image, factor);
      }

      if (p == "-dsm")
      {
        int factor;

        param.nextValue(factor);
        image=medianDownscaleImage(image, factor);
      }

      if (p == "-dsg")
      {
        gimage::Image<T> tmp;
        gimage::reduceGauss(tmp, image);
        image=tmp;
      }

      if (p == "-usg")
      {
        gimage::Image<T> tmp;
        gimage::expandGauss(tmp, image);
        image=tmp;
      }

      if (p == "-crop")
      {
        long x, y, w, h;

        param.nextValue(x);
        param.nextValue(y);
        param.nextValue(w);
        param.nextValue(h);
        image=cropImage(image, x, y, w, h);
      }

      if (p == "-u8")
      {
        gimage::ImageU8 imageu8;
        imageu8.setImageLimited(image);
        image.setSize(0, 0, 0);
        process(imageu8, param, repl);
        break;
      }

      if (p == "-u16")
      {
        gimage::ImageU16 imageu16;
        imageu16.setImageLimited(image);
        image.setSize(0, 0, 0);
        process(imageu16, param, repl);
        break;
      }

      if (p == "-float")
      {
        gimage::ImageFloat imagef;
        imagef.setImageLimited(image);
        image.setSize(0, 0, 0);
        process(imagef, param, repl);
        break;
      }

      if (p == "-select")
      {
        gimage::Image<T> tmp;
        std::string   channel;

        param.nextString(channel, "I|R|G|B");

        if (channel == "I")
        {
          imageToGrey(tmp, image);
        }
        else if (channel == "R")
        {
          imageSelect(tmp, image, 0);
        }
        else if (channel == "G")
        {
          imageSelect(tmp, image, 1);
        }
        else if (channel == "B")
        {
          imageSelect(tmp, image, 2);
        }

        image=tmp;
      }

      if (p == "-color")
      {
        gimage::Image<T> tmp;

        imageToColor(tmp, image);

        image=tmp;
      }

      if (p == "-jet")
      {
        gimage::ImageU8 image8;

        gimage::imageToJET(image8, image);
        process(image8, param, repl);
        break;
      }

      if (p == "-rgb2hsv")
      {
        gimage::ImageFloat imagef;

        rgbToHSV(imagef, image);
        image.setSize(0, 0, 0);
        process(imagef, param, repl);
        break;
      }

      if (p == "-hsv2rgb8")
      {
        gimage::ImageU8 imageu8;
        gimage::ImageFloat imagef;

        imagef.setImageLimited(image);
        image.setSize(0, 0, 0);

        hsvToRGB(imageu8, imagef);
        imagef.setSize(0, 0, 0);
        process(imageu8, param, repl);
        break;
      }

      if (p == "-hist")
      {
        gimage::Histogram hist(image);

        for (int i=0; i<hist.getWidth(); i++)
        {
          std::cout << i << " " << hist(i) << std::endl;
        }
      }

      if (p == "-histimage")
      {
        gimage::Histogram hist(image);
        gimage::ImageU8   himage;

        hist.visualize(himage);
        image.setSize(0, 0, 0);
        process(himage, param, repl);
        break;
      }

      if (p == "-corrhist")
      {
        gimage::Image<T> image2;

        gimage::getImageIO().load(image2, nextParameterFilename(param, repl).c_str());

        gimage::Histogram hist(image, image2);
        gimage::ImageU8   himage;

        hist.visualize(himage);
        image.setSize(0, 0, 0);
        image2.setSize(0, 0, 0);
        process(himage, param, repl);
        break;
      }

      if (p == "-gamma")
      {
        gimage::Image<T> map;
        double s;

        param.nextValue(s);
        fillGammaMap(map, image, s);
        remapImage(image, map);
      }

      if (p == "-add")
      {
        double s;

        param.nextValue(s);
        image+=static_cast<typename gimage::Image<T>::work_t>(s);
      }

      if (p == "-sub")
      {
        double s;

        param.nextValue(s);
        image-=static_cast<typename gimage::Image<T>::work_t>(s);
      }

      if (p == "-mul")
      {
        double s;

        param.nextValue(s);
        image*=s;
      }

      if (p == "-div")
      {
        double s;

        param.nextValue(s);
        image/=s;
      }

      if (p == "-reciprocal")
      {
        reciprocal(image);
      }

      if (p == "-addimage")
      {
        gimage::Image<T> image2;

        gimage::getImageIO().load(image2, nextParameterFilename(param, repl).c_str());

        image+=image2;
      }

      if (p == "-subimage")
      {
        gimage::Image<T> image2;

        gimage::getImageIO().load(image2, nextParameterFilename(param, repl).c_str());

        image-=image2;
      }

      if (p == "-gauss")
      {
        float s;

        param.nextValue(s);

        gimage::gauss(image, s);
      }

      if (p == "-noise")
      {
        double s;
        param.nextValue(s);

        srand(static_cast<unsigned int>(1000000*gutil::ProcTime::current()));

        addScaledNoise(image, static_cast<float>(s));
      }

      if (p == "-valid")
      {
        double from, to;

        param.nextValue(from);
        param.nextValue(to);
        validRange(image, static_cast<T>(from), static_cast<T>(to));
      }

      if (p == "-clip")
      {
        double from, to;

        param.nextValue(from);
        param.nextValue(to);
        clipRange(image, static_cast<T>(from), static_cast<T>(to));
      }

      if (p == "-hdr")
      {
        float scale;
        param.nextValue(scale);

        gimage::HighDynamicRangeFusion<T> hdr;

        hdr.add(image, std::min(65535.0f, static_cast<float>(image.absMaxValue())));

        std::string s;
        std::vector<std::string> slist;
        param.nextString(s);
        gutil::split(slist, s, ',');

        for (size_t i=0; i<slist.size(); i++)
        {
          gimage::Image<T> tmp;
          std::string name=slist[i];
          gutil::trim(name);
          gimage::getImageIO().load(tmp, name.c_str());
          hdr.add(tmp, std::min(65535.0f, static_cast<float>(image.absMaxValue())));
        }

        hdr.fuse(image, scale);
      }

      if (p == "-cmp")
      {
        gimage::Image<T> image2;
        gimage::getImageIO().load(image2, nextParameterFilename(param, repl).c_str());

        std::string s;
        std::vector<std::string> slist;
        param.nextString(s);
        gutil::split(slist, s, ',');

        std::vector<T> tol;

        for (size_t i=0; i<slist.size(); i++)
        {
          typename gimage::Image<T>::work_t v;
          std::istringstream in(slist[i]);
          in >> v;

          tol.push_back(static_cast<T>(v));
        }

        double tf;
        param.nextValue(tf);

        gimage::Image<T> diff;

        long outlier=gimage::cmp(diff, image, image2, tol);
        image=diff;

        double f=100*static_cast<double>(outlier)/
                 (image.getWidth()*image.getHeight()*image.getDepth());

        if (outlier >= 0 && f <= tf)
        {
          std::cout << "Images are the same within given tolerances." << std::endl;
        }
        else if (outlier < 0)
        {
          std::cout << "Size or number of color channels differs!" << std::endl;
        }
        else
          std::cout << "Image differences exceed given tolerances. Oulier: "
                    << std::setprecision(5) << f << " %" << std::endl;
      }

      if (p == "-paste")
      {
        gimage::Image<T> image2;
        long     x, y;
        int      z;
        float    opacity;

        gimage::getImageIO().load(image2, nextParameterFilename(param, repl).c_str());
        param.nextValue(x);
        param.nextValue(y);
        param.nextValue(z);
        param.nextValue(opacity);

        paste(image, image2, x, y, z, opacity);
      }

      if (p == "-pick")
      {
        long x, y;

        param.nextValue(x);
        param.nextValue(y);

        std::cout << "image(" << x << ", " << y << "):";

        for (int d=0; d<image.getDepth(); d++)
        {
          std::cout << " " << image.getW(x, y, d);
        }

        std::cout << std::endl;
      }

      if (p == "-print")
      {
        std::string what;
        param.nextString(what, "all|type|min|max|mean|width|height|depth");

        if (what == "all" || what == "type")
        {
          typedef typename gimage::Image<T>::ptraits ptraits;
          std::cout << "type=" << ptraits::description() << std::endl;
        }

        if (what == "all" || what == "min")
        {
          std::cout << "min=" << image.minValue() << std::endl;
        }

        if (what == "all" || what == "max")
        {
          std::cout << "max=" << image.maxValue() << std::endl;
        }

        if (what == "all" || what == "mean")
        {
          double v=0;

          for (long k=0; k<image.getHeight(); k++)
          {
            for (long i=0; i<image.getWidth(); i++)
            {
              v+=image.get(i, k);
            }
          }

          v/=image.getWidth()*image.getHeight();

          std::cout << "mean=" << v << std::endl;
        }

        if (what == "all" || what == "width")
        {
          std::cout << "width=" << image.getWidth() << std::endl;
        }

        if (what == "all" || what == "height")
        {
          std::cout << "height=" << image.getHeight() << std::endl;
        }

        if (what == "all" || what == "depth")
        {
          std::cout << "depth=" << image.getDepth() << std::endl;
        }
      }
    }
  }
  catch (gutil::Exception &ex)
  {
    std::cerr << ex.what() << std::endl;
    std::cerr << ex.where() << std::endl;
  }
  catch (std::exception &ex)
  {
    std::cerr << ex.what() << std::endl;
  }
}

}

int main(int argc, char *argv[])
{
  // command line definition

  const char *def[]=
  {
    "# imgcmd [-help | -version] <in> [<options>]",
    "#",
    "# The input and output names may contain the wildcard '%'.",
    "#",

    "-help # Print help and exit.",

    "-version # Print version and exit.",

    "#",
    "# All options are processed strictly in the order in which they appear on the command line. All options may appear multiple times. Options are:",
    "#",

    "-out # Stores the image. The image format depends on the suffix.",
    " <name> # File name.",

    "-ds # Downscaling the image by computing the mean.",
    " <ds> # Integer downscale factor.",

    "-dsm # Downscaling the image by computing the median.",
    " <ds> # Integer downscale factor.",

    "-dsg # Downscaling the image by factor 2 after Gaussian smoothing.",

    "-usg # Upsampling the image by factor 2 after Gaussian smoothing.",

    "-crop # Selecting a part of the image.",
    " <x> <y> # Left upper corner of the image.",
    " <w> <h> # Width and height of the image.",

    "-u8 # Converts the image to 8 bit unsigned integer. Larger values are saturated.",
    "-u16 # Converts the image to 16 bit unsigned integer. Larger values are saturated.",
    "-float # Converts the image to floating point format.",

    "-select # Selects a color channel for an intensity image.",
    " I|R|G|B # Channel, I means intensity.",

    "-color # Makes a color image from an intensity image by putting the value into R, G and B.",

    "-jet # Makes a color image from an intensity image using JET encoding.",

    "-rgb2hsv # Converts an image from HSV to RGB.",

    "-hsv2rgb8 # Converts an image from RGB to HSV.",

    "-hist # Computes the histogram and prints it to stdout.",

    "-histimage # Computes the histogram as image.",

    "-corrhist # Computes the correspondence histogram with the second image.",
    " <image2> # File name of second image in the same format as the first image.",

    "-gamma # Gamma transformation.",
    " <s> # Gamma factor.",

    "-add # Add an offset to all pixels.",
    " <s> # Offset value of the same type than the pixel values of the image.",

    "-sub # Subtract an offset from all pixels.",
    " <s> # Offset value of the same type than the pixel values of the image.",

    "-mul # Multiplies all pixels by the given value.",
    " <s> # Floating point value.",

    "-div # Divides all pixels by the given value.",
    " <s> # Floating point value.",

    "-reciprocal # Computes the reciprocal of all pixel values",

    "-addimage # Adds pixel values of the given image to the input image.",
    " <image2> # File name of second image in the same format and size as the first image.",

    "-subimage # Subtracts pixel values of the given image from the input image.",
    " <image2> # File name of second image in the same format and size as the first image.",

    "-gauss # Gaussian smoothing.",
    " <s> # Standard deviation, which must be >= 0.5.",

    "-noise # Add Gaussian noise to the pixel values. The noise is scaled to the pixel intensity.",
    " <s> # Standard deviation at full intensity.",

    "-valid # Invalidates pixels outside the given range.",
    " <from> <to> # Range.",

    "-clip # Sets smaller or larger pixel values to the corresponding range value.",
    " <from> <to> # Range.",

    "-hdr # Fuses all given image into one high dynamic range image.",
    " <scale> # Factor for scaling pixel values. Default: 1.0",
    " <image 1>,... # Comma separated list of images.",

    "-cmp # Compares images with some tolerance. The result is printed and given as image with absolute differences.",
    " <image> # Image for comparison.",
    " <t0, ... tn> # Comma separated list of tolerances per color channel. The last value is used if there are more channels.",
    " <f> # Outliers in percent, i.e. pixel with completely different values.",

    "-paste # Pasts another image into the image.",
    " <name> # File name of second image.",
    " <x> <y> <z> # Position for in the first image to start the second image.",
    " <opacity> # Factor between 0 and 1 that defines the opacity of the new image.",

    "-pick # Prints the value of the pixel to stdout.",
    " <x> <y> # Position of pixel.",

    "-print # Prints information about the image to stdout.",
    " <what> # Kind of requested information: all, min, max, mean, width, height, depth or type.",

    0
  };

  gutil::Parameter param(argc, argv, def);

  if (param.remaining() < 1)
  {
    param.printHelp(std::cout);
    return 10;
  }

  // check if the first two parameters are -help or -version

  std::string p;

  if (param.isNextParameter())
  {
    param.nextParameter(p);

    if (p == "-help")
    {
      param.printHelp(std::cout);
      return 0;
    }
    else if (p == "-version")
    {
      std::cout << "This program is part of cvkit version " << VERSION << std::endl;
      return 0;
    }
    else
    {
      std::cerr << "The first parameter must be the image file name!" << std::endl;
      return 10;
    }
  }

  // get name

  std::string prefix, suffix;

  param.nextString(prefix);

  // read first two parameters, if they are -ds and -crop,
  // because this can be considered while loading

  std::set<std::string> list;
  int  ds=1;
  long x=0, y=0, w=-1, h=-1;

  if (param.remaining() > 0)
  {
    param.nextParameter(p);

    if (p == "-ds")
    {
      param.nextValue(ds);
      ds=std::max(1, ds);
    }
    else
    {
      param.previous();
    }
  }

  if (param.remaining() > 0)
  {
    param.nextParameter(p);

    if (p == "-crop")
    {
      param.nextValue(x);
      param.nextValue(y);
      param.nextValue(w);
      param.nextValue(h);
    }
    else
    {
      param.previous();
    }
  }

  // determine list of files if the pattern character '%' is found in the
  // file name

  size_t pos=prefix.find('%');

  if (pos < prefix.size())
  {
    suffix=prefix.substr(pos+1);
    prefix=prefix.substr(0, pos);
    gutil::getFileList(list, prefix, suffix);
  }
  else
  {
    list.insert(prefix);
  }

  // try loading with increasing data type and start processing using
  // remainder of the command line

  for (std::set<std::string>::iterator it=list.begin(); it!=list.end(); ++it)
  {
    if (list.size() > 1)
    {
      std::cout << *it << ":" << std::endl;
    }

    std::string repl=it->substr(prefix.size(), it->size()-prefix.size()-
                                suffix.size());

    try
    {
      gimage::ImageU8 image;

      gimage::getImageIO().load(image, it->c_str(), ds, x, y, w, h);
      process(image, param, repl);
    }
    catch (const std::exception &)
    {
      try
      {
        gimage::ImageU16 image;

        gimage::getImageIO().load(image, it->c_str(), ds, x, y, w, h);
        process(image, param, repl);
      }
      catch (const std::exception &)
      {
        try
        {
          gimage::ImageFloat image;

          gimage::getImageIO().load(image, it->c_str(), ds, x, y, w, h);
          process(image, param, repl);
        }
        catch (const gutil::Exception &ex)
        {
          ex.print();
        }
      }
    }
  }

  return 0;
}
