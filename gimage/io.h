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

#ifndef GIMAGE_IO_H
#define GIMAGE_IO_H

#include "image.h"

#include <gutil/properties.h>
#include <gutil/exception.h>

#include <vector>

namespace gimage
{

/**
 * Virtual base class for loading and saving images. The sub-classes implement
 * loading and saving for different file formats.
 *
 * For loading images:
 *
 * If the stored pixel type is equal or smaller compared to the given image,
 * then its pixel values are casted into the larger type while loading.
 * Otherwise, an exception is thrown.
 *
 * Thread safety:
 *
 * Do not use local variables in child classes! That is why there is also no
 * virtual destructor required.
 */

class BasicImageIO
{
  public:

    virtual ~BasicImageIO() {};

    virtual BasicImageIO *create() const=0;

    virtual bool handlesFile(const char *name, bool reading) const=0;

    virtual void loadHeader(const char *name, long &width, long &height, int &depth) const;

    virtual void loadProperties(gutil::Properties &prop, const char *name) const ;
    virtual void load(ImageU8 &image, const char *name, int ds=1, long x=0, long y=0, long w=-1,
                      long h=-1) const;
    virtual void load(ImageU16 &image, const char *name, int ds=1, long x=0, long y=0, long w=-1,
                      long h=-1) const;
    virtual void load(ImageFloat &image, const char *name, int ds=1, long x=0, long y=0, long w=-1,
                      long h=-1) const;

    virtual void saveProperties(const gutil::Properties &prop, const char *name) const;
    virtual void save(const ImageU8 &image, const char *name) const;
    virtual void save(const ImageU16 &image, const char *name) const;
    virtual void save(const ImageFloat &image, const char *name) const;
};

/**
 * Functions for loading and saving images in different formats. The format is
 * determined by the file name (e.g. suffix).
 *
 * For loading images:
 *
 * The class handles tiled files if the image file name has the format
 * <prefix>:<suffix>. The tiles must have the same size and image format and
 * named as <prefix>_<row number>_<column number>_<suffix>.
 *
 * Thread safety:
 *
 * Add BasicImageIO objects before starting additional threads.
 */

class ImageIO
{
  public:

    ImageIO();

    void addBasicImageIO(const BasicImageIO &io);

    bool handlesFile(const char *name, bool reading) const;

    void loadHeader(const char *name, long &width, long &height, int &depth) const;

    void loadProperties(gutil::Properties &prop, const char *name) const;
    void load(ImageU8 &image, const char *name, int ds=1, long x=0, long y=0, long w=-1,
              long h=-1) const;
    void load(ImageU16 &image, const char *name, int ds=1, long x=0, long y=0, long w=-1,
              long h=-1) const;
    void load(ImageFloat &image, const char *name, int ds=1, long x=0, long y=0, long w=-1,
              long h=-1) const;

    void saveProperties(const gutil::Properties &prop, const char *name) const;
    void save(const ImageU8 &image, const char *name) const;
    void save(const ImageU16 &image, const char *name) const;
    void save(const ImageFloat &image, const char *name) const;

  private:

    const BasicImageIO &getBasicImageIO(const char *name, bool reading) const;

    std::vector<BasicImageIO *> list;
};

/**
 * Returns the global ImageIO object for loading and saving images.
 */

ImageIO &getImageIO();

/**
 * Returns an unused filename for storing an image. The name is derived by
 * splitting the existing name (which may include a directory) into a prefix
 * and suffix and putting an increasing integer number in between. If the name
 * does not include a suffix, then the full name is taken as prefix and a
 * suffix is internally created. This function returns an empty std::string, if a
 * new name cannot be generated.
 */

std::string getNewImageName(std::string prefix);

}

#endif