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

#include "listimagewindow.h"

#include <gutil/exception.h>

using std::string;
using std::ostringstream;

using gimage::ImageU8;
using gimage::ImageU16;
using gimage::ImageFloat;

namespace bgui
{

namespace
{

string createHelpText()
{
    ostringstream out;
    
    out << "\n";
    out << "List:\n";
    out << "<cursor left>  Load previous image\n";
    out << "<cursor right> Load next image\n";
    out << "'k'            Switch between keep, keep_all and not keeping the settings when changing between images\n";
    
    return out.str();
}

}

void ListImageWindow::set(int pos, int w, int h, bool size_max)
{
    if (pos >= 0 && pos < static_cast<int>(list.size()))
    {
      ImageAdapterBase *adapt=list[pos];
      
      if (imin < imax && kp == keep_all)
      {
        adapt->setMinIntensity(imin);
        adapt->setMaxIntensity(imax);
      }
      
      if (w > 0 && h > 0)
      {
        adapt->setMapping(map);
        adapt->setChannel(channel);
        setAdapter(adapt, false, keep_none, w, h, size_max);
      }
      else
      {
        if (kp == keep_none)
        {
          adapt->setMapping(map);
          adapt->setChannel(channel);
        }
        
        setAdapter(adapt, false, kp);
      }
    }
    
    updateTitle();
}

void ListImageWindow::updateTitle()
{
    ImageAdapterBase *adapt=getAdapter();
    
      // set image adapter and title
    
    if (adapt != 0)
    {
      ostringstream os;
      
      os << name[current] << " - " << adapt->getOriginalWidth() << "x" <<
        adapt->getOriginalHeight() << "x" << adapt->getOriginalDepth() <<
        " " << adapt->getOriginalType();
      
      if (kp != keep_none)
      {
        os << " -";
        
        if (kp == keep_most)
          os << " " << "keep";
        
        if (kp == keep_all)
          os << " " << "keep_all";
      }
      
      setTitle(os.str().c_str());
      setInfoText("");
    }
    else
    {
      setAdapter(0);
      setTitle("Image");
      setInfoText("No image!");
    }
}

ListImageWindow::ListImageWindow(double init_min, double init_max,
  double valid_min, double valid_max, keep k, mapping m, int c)
{
    addHelpText(createHelpText());
    
    current=0;
    
    imin=init_min;
    imax=init_max;
    vmin=valid_min;
    vmax=valid_max;
    kp=k;
    map=m;
    channel=c;
    
    set(-1, 0, 0, false);
}

ListImageWindow::~ListImageWindow()
{
    for (size_t i=0; i<list.size(); i++)
      delete list[i];
}

void ListImageWindow::add(const ImageU8 &image, const string &s, bool copy)
{
    const ImageU8 *im=&image;
    
    if (copy)
      im=new ImageU8(image);
    
    ImageAdapterBase *adapt=new ImageAdapter<unsigned char>(im, vmin, vmax, copy);
    
    list.push_back(adapt);
    
    if (s.size() == 0)
    {
      ostringstream out;
      
      out << "Image " << name.size();
      
      name.push_back(out.str());
    }
    else
      name.push_back(s);
    
    if (list.size() == 1)
    {
      int w, h;
      
      getDisplaySize(w, h);
      
      current=0;
      set(current, w, h, true);
      
      setVisible(true);
    }
}

void ListImageWindow::add(const ImageU16 &image, const string &s, bool copy)
{
    const ImageU16 *im=&image;
    
    if (copy)
      im=new ImageU16(image);
    
    ImageAdapterBase *adapt=new ImageAdapter<unsigned short>(im, vmin, vmax, copy);
    
    list.push_back(adapt);
    
    if (s.size() == 0)
    {
      ostringstream out;
      
      out << "Image " << name.size();
      
      name.push_back(out.str());
    }
    else
      name.push_back(s);
    
    if (list.size() == 1)
    {
      int w, h;
      
      getDisplaySize(w, h);
      
      current=0;
      set(current, w, h, true);
      
      setVisible(true);
    }
}

void ListImageWindow::add(const ImageFloat &image, const string &s, bool copy)
{
    const ImageFloat *im=&image;
    
    if (copy)
      im=new ImageFloat(image);
    
    ImageAdapterBase *adapt=new ImageAdapter<float>(im, vmin, vmax, copy);
    
    list.push_back(adapt);
    
    if (s.size() == 0)
    {
      ostringstream out;
      
      out << "Image " << name.size();
      
      name.push_back(out.str());
    }
    else
      name.push_back(s);
    
    if (list.size() == 1)
    {
      int w, h;
      
      getDisplaySize(w, h);
      
      current=0;
      set(current, w, h, true);
      
      setVisible(true);
    }
}

void ListImageWindow::onKey(char c, SpecialKey key, int x, int y)
{
    switch (key)
    {
      case k_left: /* load previous image */
          if (current > 0)
          {
            current--;
            set(current);
          }
          
          setInfoLine("");
          break;
      
      case k_right: /* load next image */
          current++;
          if (current < static_cast<int>(name.size()))
            set(current);
          else
            current--;
          
          setInfoLine("");
          break;
      
      default:
          break;
    }
    
    switch (c)
    {
      case 'k':
          switch (kp)
          {
            case keep_none:
                kp=keep_most;
                break;
            
            case keep_most:
                kp=keep_all;
                break;
            
            case keep_all:
                kp=keep_none;
                break;
          }
          
          updateTitle();
          break;
    }
    
    ImageWindow::onKey(c, key, x, y);
}

}
