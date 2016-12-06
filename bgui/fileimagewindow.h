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

#ifndef BGUI_FILEIMAGEWINDOW_H
#define BGUI_FILEIMAGEWINDOW_H

#include "imagewindow.h"

#include <string>
#include <vector>
#include <limits>

namespace bgui
{

/**
 * Simple window class.
 */

class FileImageWindow : public ImageWindow
{
  private:

    std::string  vc;

    std::vector<std::string> list;
    unsigned int   current;

    double  scale;
    double  imin, imax;
    double  vmin, vmax;
    keep    kp;
    mapping map;
    int     channel;
    bool    watch_file;
    int     wid;

    void load(unsigned int &pos, bool down=true, int w=-1, int h=-1,
      bool size_max=false);

    void updateTitle();

    void saveContent(const char *basename);

  public:

    FileImageWindow(const std::vector<std::string> &files,  int firstfile,
      bool watch=false, int x=-1, int y=-1, int w=-1, int h=-1,
      bool size_max=false, double init_scale=0, double init_min=0,
      double init_max=0, double valid_min=-std::numeric_limits<float>::max(),
      double valid_max=std::numeric_limits<float>::max(),
      keep k=keep_none, mapping m=map_raw, int c=-1, const char *viewcmd=0);
    virtual ~FileImageWindow();

    virtual void onKey(char c, SpecialKey key, int x, int y);
    virtual void onFileChanged(int watchid);
};

}

#endif
