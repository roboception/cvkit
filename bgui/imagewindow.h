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

#ifndef BGUI_IMAGEWINDOW_H
#define BGUI_IMAGEWINDOW_H

#include "basewindow.h"
#include "imageadapterbase.h"

#include <gimage/image.h>

#include <limits>

namespace bgui
{

/**
 * Simple window class.
 */

class ImageWindow : public BaseWindow
{
  private:
  
    ImageAdapterBase *adapt;
    bool             del;
    
    std::string      helptext;
    
    char             lastkey;
    int              imx, imy;
    int              xp, yp;
    bool             showinfo;
    
    void updateInfo();
    
  protected:
  
    void addHelpText(const std::string &text);
  
  public:
    
    enum keep {keep_none, keep_most, keep_all};
    
    ImageWindow();
    ImageWindow(const gimage::ImageU8 &image, int x=-1, int y=-1, int w=-1, int h=-1,
      double vmin=-std::numeric_limits<float>::max(), double vmax=std::numeric_limits<float>::max());
    
    ImageWindow(const gimage::ImageU16 &image, int x=-1, int y=-1, int w=-1, int h=-1,
      double vmin=-std::numeric_limits<float>::max(), double vmax=std::numeric_limits<float>::max());
    
    ImageWindow(const gimage::ImageFloat &image, int x=-1, int y=-1, int w=-1, int h=-1,
      double vmin=-std::numeric_limits<float>::max(), double vmax=std::numeric_limits<float>::max());
    
    virtual ~ImageWindow();
    
    void setAdapter(ImageAdapterBase *adapter, bool delete_on_close=false,
      keep k=keep_none, int w=-1, int h=-1, bool size_max=false);
    ImageAdapterBase *getAdapter();
    
    void redrawImage(bool force=true);
  
    void visibleImagePart(long &x, long &y, long &w, long &h);
    
    virtual void onResize(int w, int h);
    virtual void onMousePressed(Button b, int x, int y, int state);
    virtual void onMouseReleased(Button b, int x, int y, int state);
    virtual void onMouseMove(int x, int y, int state);
    virtual void onKey(char c, SpecialKey key, int x, int y);
    
    char getLastKey();
};

}

#endif

