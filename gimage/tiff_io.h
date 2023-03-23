/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2023, Roboception GmbH, Munich, Germany
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

#ifndef GIMAGE_TIFF_IO_H
#define GIMAGE_TIFF_IO_H

#include "io.h"

namespace gimage
{

/**
 * Supports reading and writing TIFF (<prefix>.tiff or <prefix>.tif) for
 * ImageU8, ImageU16 and ImageFloat.
 */

class TIFFImageIO : public BasicImageIO
{
  public:

    BasicImageIO *create() const;

    bool handlesFile(const char *name, bool reading) const;

    void loadHeader(const char *name, long &width, long &height, int &depth) const;

    void load(ImageU8 &image, const char *name, int ds=1, long x=0, long y=0, long w=-1,
              long h=-1) const;
    void load(ImageU16 &image, const char *name, int ds=1, long x=0, long y=0, long w=-1,
              long h=-1) const;
    void load(ImageFloat &image, const char *name, int ds=1, long x=0, long y=0, long w=-1,
              long h=-1) const;

    void save(const ImageU8 &image, const char *name) const;
    void save(const ImageU16 &image, const char *name) const;
    void save(const ImageFloat &image, const char *name) const;
};

}

#endif
