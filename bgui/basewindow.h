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

#ifndef BGUI_BASEWINDOW_H
#define BGUI_BASEWINDOW_H

#include "imageadapterbase.h"

namespace bgui
{

/**
 * Simple window class.
 */

class BaseWindow
{
  private:
  
    struct BaseWindowData *p;
    
  public:
    
    BaseWindow(const char *title, int w, int h);
    virtual ~BaseWindow();
    
      // showing and hiding the window (only works as long as it is not closed)
    
    void setVisible(bool show=true);
    
    int addFileWatch(const char *path);
    void removeFileWatch(int watchid);
    
    void sendClose();    // sends close event to the window
    void waitForClose(); // waits for the window to be closed
    bool isClosed();
    
      // capturing events
    
    enum Button {button1, button2, button3, button4, button5};
    
    const static int button1mask=1<<0;
    const static int button2mask=1<<1;
    const static int button3mask=1<<2;
    const static int button4mask=1<<3;
    const static int button5mask=1<<4;
    const static int shiftmask=1<<5;
    const static int ctrlmask=1<<6;
    
    virtual void onResize(int w, int h) { };
    virtual void onMousePressed(Button b, int x, int y, int state) { };
    virtual void onMouseReleased(Button b, int x, int y, int state) { };
    virtual void onMouseMove(int x, int y, int state) { };
    virtual void onFileChanged(int watchid) { };
    
    enum SpecialKey {k_none, k_esc, k_left, k_right, k_up, k_down};
    
    virtual void onKey(char c, SpecialKey key, int x, int y) { };
    
      // return true for allowing the window to close
    
    virtual bool onClose() { return true; }
    
      // stores the current content of the window into the given image
    
    void getContent(gimage::ImageU8 &image);
    
  protected:
    
    void setTitle(const char *title);
    void getDisplaySize(int &w, int &h);
    
      // get and set the window size excluding border
    
    void getSize(int &w, int &h);
    void setSize(int w, int h);
    void setPosition(int x, int y);
    int  getTextHeight();
    
      // sets or resets one line of text, which is immediately shown
      // (set an empty string for removing the info text)
    
    void setInfoLine(const char *text, bool top=true, bool left=true);
    
      // sets or resets a multi line text, which is immediately shown
      // (set an empty string for removing the info text)
    
    void setInfoText(const char *text);
    bool hasInfoText();
    
      // clear and paint images into background buffer and visualize it
    
    void clearBuffer();
    void paintBuffer(const ImageAdapterBase &image, int x=0, int y=0);
    void showBuffer();
};

}

#endif

