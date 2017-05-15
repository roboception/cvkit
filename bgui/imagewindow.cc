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

#include "imagewindow.h"
#include "imageadapter.h"

#include <gutil/version.h>

#include <algorithm>

#ifdef INCLUDE_PNG
#include <png.h>
#endif

namespace bgui
{

namespace
{

std::string createHelpText()
{
  std::ostringstream out;

  out << "Usage and Keycodes:\n";
  out << "\n";
  out << "left mouse click and moving for panning\n";
  out << "right mouse click (or shift and left mouse click) for pixel information\n";
  out << "mouse wheel for zooming\n";
  out << "\n";
  out << "Information and Exit:\n";
  out << "'h'            Shows this help text.\n";
  out << "'v'            Shows version information.\n";
  out << "'i'            Shows information about the currently viewed image part.\n";
  out << "'q', 'esc'     Exit\n";
  out << "\n";
  out << "Select color channel:\n";
  out << "'C'            Reset color channel selection.\n";
  out << "'R', 'G', 'B'  Shows only the corresponding color channel.\n";
  out << "\n";
  out << "Size and Orientation:\n";
  out << "'0'            (Re)set zoom factor to 1.\n";
  out << "'+', '-'       Zooming in and out.\n";
  out << "'s'            Resize window to size of image at current scaling\n";
  out << "'r', 'l'       Rotate anti-clockwise or clockwise by 90 degrees\n";
  out << "'f'            Flip image horizontally.\n";
  out << "\n";
  out << "Radiometry:\n";
  out << "'t'            Switches smoothing by trilinear filtering on or off.\n";
  out << "'a'            Scale such that currently visible part of the image uses full contrast\n";
  out << "'b', 'w'       Scale such that pixel at current mouse position is black or white\n";
  out << "'m'            Switching between raw, jet and rainbow mapping for greyscale images\n";
  out << "'1', '2', '4'  Setting the corresponding value as gamma factor\n";

  return out.str();
}

}

void ImageWindow::updateInfo()
{
  if (adapt != 0 && showinfo)
  {
    int           w, h;
    double        scale=adapt->getScale();
    std::ostringstream os;

    getSize(w, h);

    os << "rotation=" << adapt->getRotation()*90;
    os << "; flip=" << adapt->getFlip();
    os << "; scale=" << scale;
    os << "; x=" << static_cast<long>(std::max(imx, 0)/scale);
    os << ", y=" << static_cast<long>(std::max(imy, 0)/scale);
    os << ", w=" << static_cast<long>(std::min(static_cast<long>(w), adapt->getWidth())/scale);
    os << ", h=" << static_cast<long>(std::min(static_cast<long>(h), adapt->getHeight())/scale);
    os << "; range=[" << adapt->getMinIntensity();
    os << ":" << adapt->getMaxIntensity();
    os << "]; smooth=" << adapt->getSmoothing();
    os << "; gamma=" << adapt->getGamma();

    setInfoLine(os.str().c_str(), true, true);
  }
  else
  {
    setInfoLine("");
  }
}

void ImageWindow::addHelpText(const std::string &text)
{
  helptext.append(text);
}

ImageWindow::ImageWindow() : BaseWindow("Image", 100, 100)
{
  adapt=0;
  del=false;
  lastkey='\0';
  imx=imy=0;
  xp=yp=0;
  showinfo=false;

  addHelpText(createHelpText());
};

ImageWindow::ImageWindow(const gimage::ImageU8 &image, int x, int y, int w, int h,
                         double vmin, double vmax) : BaseWindow("Image", 100, 100)
{
  ImageAdapterBase *p=new ImageAdapter<gutil::uint8>(&image, vmin, vmax);

  adapt=0;
  del=false;
  lastkey='\0';
  imx=imy=0;
  showinfo=false;

  addHelpText(createHelpText());

  std::ostringstream os;

  os << "Image - " << image.getWidth() << "x" << image.getHeight() << "x" <<
     image.getDepth() << " " << image.getTypeDescription();

  setTitle(os.str().c_str());

  if (w <= 0)
  {
    w=image.getWidth();
  }

  if (h <= 0)
  {
    h=image.getHeight();
  }

  setAdapter(p, true, keep_none, w, h);
  setVisible(true);

  if (x >= 0 && y >= 0)
  {
    setPosition(x, y);
  }
}

ImageWindow::ImageWindow(const gimage::ImageU16 &image, int x, int y, int w, int h,
                         double vmin, double vmax) : BaseWindow("Image", 100, 100)
{
  ImageAdapterBase *p=new ImageAdapter<gutil::uint16>(&image, vmin, vmax);

  adapt=0;
  del=false;
  lastkey='\0';
  imx=imy=0;
  showinfo=false;

  addHelpText(createHelpText());

  std::ostringstream os;

  os << "Image - " << image.getWidth() << "x" << image.getHeight() << "x" <<
     image.getDepth() << " " << image.getTypeDescription();

  setTitle(os.str().c_str());

  if (w <= 0)
  {
    w=image.getWidth();
  }

  if (h <= 0)
  {
    h=image.getHeight();
  }

  setAdapter(p, true, keep_none, w, h);
  setVisible(true);

  if (x >= 0 && y >= 0)
  {
    setPosition(x, y);
  }
}

ImageWindow::ImageWindow(const gimage::ImageFloat &image, int x, int y, int w, int h,
                         double vmin, double vmax) : BaseWindow("Image", 100, 100)
{
  ImageAdapterBase *p=new ImageAdapter<float>(&image, vmin, vmax);

  adapt=0;
  del=false;
  lastkey='\0';
  imx=imy=0;
  showinfo=false;

  addHelpText(createHelpText());

  std::ostringstream os;

  os << "Image - " << image.getWidth() << "x" << image.getHeight() << "x" <<
     image.getDepth() << " " << image.getTypeDescription();

  setTitle(os.str().c_str());

  if (w <= 0)
  {
    w=image.getWidth();
  }

  if (h <= 0)
  {
    h=image.getHeight();
  }

  setAdapter(p, true, keep_none, w, h);
  setVisible(true);

  if (x >= 0 && y >= 0)
  {
    setPosition(x, y);
  }
}

ImageWindow::~ImageWindow()
{
  if (del)
  {
    delete adapt;
  }
}

void ImageWindow::setAdapter(ImageAdapterBase *adapter, bool delete_on_close,
                             keep k, int w, int h, bool size_max)
{
  showinfo=false;
  updateInfo();

  if (adapter != 0 && adapt != 0)
  {
    if (k == keep_most || k == keep_all)
    {
      adapter->setChannel(adapt->getChannel());
      adapter->setScale(adapt->getScale());
      adapter->setGamma(adapt->getGamma());
      adapter->setMapping(adapt->getMapping());
      adapter->setSmoothing(adapt->getSmoothing());
    }

    if (k == keep_all)
    {
      adapter->setMinIntensity(adapt->getMinIntensity());
      adapter->setMaxIntensity(adapt->getMaxIntensity());
    }
  }

  // delete current adapter

  if (del)
  {
    delete adapt;
  }

  // store adapter

  adapt=adapter;
  del=delete_on_close;

  if (adapt != 0)
  {
    if (k == keep_none)
    {
      bool set_window_size=false;

      if (w >= 0 && h >= 0)
      {
        set_window_size=true;
      }
      else
      {
        getSize(w, h);
      }

      int  dw, dh;

      getDisplaySize(dw, dh);

      w=std::min(w, dw);
      h=std::min(h, dh);

      // define scale for viewing

      if (adapt->getScale() == 0)
      {
        adapt->setScale(1.0);
        long iw=adapt->getWidth();
        long ih=adapt->getHeight();

        double s=static_cast<double>(w)/iw;
        adapt->setScale(s);

        if (adapt->getHeight() > h)
        {
          s=static_cast<double>(h)/ih;
          adapt->setScale(s);
        }

        if (s > 1)
        {
          adapt->setScale(1);
        }
      }

      if (set_window_size)
      {
        if (size_max)
        {
          setSize(adapt->getWidth(), adapt->getHeight());
        }
        else
        {
          setSize(w, h);
        }
      }

      imx=0;
      imy=0;
    }

    // draw image

    redrawImage();
  }
  else
  {
    clearBuffer();
    showBuffer();
  }
}

ImageAdapterBase *ImageWindow::getAdapter()
{
  return adapt;
}

void ImageWindow::redrawImage(bool force)
{
  if (adapt != 0)
  {
    int w, h;
    int x=imx, y=imy;

    // check position and modify if needed

    getSize(w, h);

    if (adapt->getWidth() >= w)
    {
      x=std::max(x, 0);

      if (adapt->getWidth()-x < w)
      {
        x=adapt->getWidth()-w;
      }
    }
    else
    {
      x=(adapt->getWidth()-w)/2;
    }

    if (adapt->getHeight() >= h)
    {
      y=std::max(y, 0);

      if (adapt->getHeight()-y < h)
      {
        y=adapt->getHeight()-h;
      }
    }
    else
    {
      y=(adapt->getHeight()-h)/2;
    }

    // paint only if forced or position has changed

    if (force || x != imx || y != imy)
    {
      imx=x;
      imy=y;

      clearBuffer();
      paintBuffer(*adapt, -imx, -imy);
      showBuffer();
    }
  }
}

void ImageWindow::visibleImagePart(long &x, long &y, long &w, long &h)
{
  if (adapt != 0)
  {
    int ww, hh;
    getSize(ww, hh);

    double scale=adapt->getScale();

    gmath::SVector<long, 3> q1, q2;

    q1[0]=static_cast<long>(std::max(imx, 0)/scale);
    q1[1]=static_cast<long>(std::max(imy, 0)/scale);
    q1[2]=1;

    q2[0]=q1[0]+static_cast<long>(std::min(static_cast<long>(ww), adapt->getWidth())/scale);
    q2[1]=q1[1]+static_cast<long>(std::min(static_cast<long>(hh), adapt->getHeight())/scale);
    q2[2]=1;

    const gmath::SVector<long, 2> p1=adapt->getRotationMatrix()*q1;
    const gmath::SVector<long, 2> p2=adapt->getRotationMatrix()*q2;

    x=std::min(p1[0], p2[0]);
    y=std::min(p1[1], p2[1]);
    w=std::max(p1[0], p2[0])-x;
    h=std::max(p1[1], p2[1])-y;
  }
  else
  {
    x=y=0;
    w=h=-1;
  }
}

void ImageWindow::onResize(int w, int h)
{
  redrawImage(false);
  updateInfo();
}

void ImageWindow::onMousePressed(Button b, int x, int y, int state)
{
  switch (b)
  {
    case button1:
      // store for panning

      xp=x+imx;
      yp=y+imy;

      if ((state & shiftmask) != 0 && adapt != 0 && xp >= 0 &&
          xp < adapt->getWidth() && yp >= 0 && yp < adapt->getHeight())
      {
        showinfo=false;
        setInfoLine(adapt->getDescriptionOfPixel(xp, yp).c_str(), y > getTextHeight());
      }
      else
      {
        updateInfo();
      }

      break;

    case button3:
      xp=x+imx;
      yp=y+imy;

      if (adapt != 0 && xp >= 0 && xp < adapt->getWidth() && yp >= 0 &&
          yp < adapt->getHeight())
      {
        showinfo=false;
        setInfoLine(adapt->getDescriptionOfPixel(xp, yp).c_str(), y > getTextHeight());
      }

      break;

    case button4:
      setInfoLine("");

      if (adapt != 0)
      {
        double s=adapt->getScale();
        double xx=(x+imx)/s;
        double yy=(y+imy)/s;

        s=1.0/pow(2.0, floor(log(1.0/s)/log(2.0)));

        if (s == adapt->getScale())
        {
          s*=2;
        }

        adapt->setScale(s);

        imx=static_cast<long>(xx*s-x);
        imy=static_cast<long>(yy*s-y);

        redrawImage();
        updateInfo();
      }

      break;

    case button5:
      if (adapt != 0)
      {
        double s=adapt->getScale();
        double xx=(x+imx)/s;
        double yy=(y+imy)/s;
        int    w, h;
        long   iw, ih;

        adapt->setScale(1.0);
        iw=adapt->getWidth();
        ih=adapt->getHeight();

        getSize(w, h);

        s=1.0/pow(2.0, floor(log(1.0/s)/log(2.0)+1e-6));

        s/=2;

        adapt->setScale(s);

        if (s < 1)
        {
          if (adapt->getWidth() < w && adapt->getHeight() < h)
          {
            s=static_cast<double>(w)/iw;
            adapt->setScale(s);

            if (adapt->getHeight() > h)
            {
              s=static_cast<double>(h)/ih;
              adapt->setScale(s);
            }
          }

          if (s > 1)
          {
            s=1;
            adapt->setScale(s);
          }
        }

        imx=static_cast<long>(xx*s-x);
        imy=static_cast<long>(yy*s-y);

        redrawImage();
        updateInfo();
      }

      break;

    default:
      break;
  }
}

void ImageWindow::onMouseReleased(Button b, int x, int y, int state)
{ }

void ImageWindow::onMouseMove(int x, int y, int state)
{
  // if left button is pressed, then perform panning

  if (state == button1mask)
  {
    imx=xp-x;
    imy=yp-y;

    redrawImage();
    updateInfo();
  }

  if ((state == (button1mask|shiftmask) || state == button3mask) &&
      adapt != 0 && x+imx >= 0 && x+imx < adapt->getWidth() && y+imy >= 0 &&
      y+imy < adapt->getHeight())
  {
    showinfo=false;
    setInfoLine(adapt->getDescriptionOfPixel(x+imx, y+imy).c_str(),
                y > getTextHeight());
  }
}

void ImageWindow::onKey(char c, SpecialKey key, int x, int y)
{
  lastkey=c;

  if (c != 'h' && c != 'v')
  {
    setInfoText("");
  }

  switch (c)
  {
    case '0':
      if (adapt != 0)
      {
        adapt->setScale(1);
        redrawImage();
        updateInfo();
      }

      break;

    case '1':
    case '2':
    case '4':
      if (adapt != 0)
      {
        adapt->setGamma(c-'1'+1);
        redrawImage();
        updateInfo();
      }

      break;

    case 'R':
      if (adapt != 0)
      {
        adapt->setChannel(0);
      }

      redrawImage();
      break;

    case 'G':
      if (adapt != 0)
      {
        adapt->setChannel(1);
      }

      redrawImage();
      break;

    case 'B':
      if (adapt != 0)
      {
        adapt->setChannel(2);
      }

      redrawImage();
      break;

    case 'C':
      if (adapt != 0)
      {
        adapt->setChannel(-1);
      }

      redrawImage();
      break;

    case '+':
      onMousePressed(button4, x, y, button4mask);
      break;

    case '-':
      onMousePressed(button5, x, y, button5mask);
      break;

    case 't':
      if (adapt != 0)
      {
        adapt->setSmoothing(!adapt->getSmoothing());
        redrawImage();
        updateInfo();
      }

      break;

    case 'a':
      if (adapt != 0)
      {
        int w, h;

        getSize(w, h);

        adapt->adaptMinMaxIntensity(imx, imy, w, h);
        redrawImage();
        updateInfo();
      }

      break;

    case 'b':
      if (adapt != 0)
      {
        int w, h;

        getSize(w, h);

        if (x+imx >= 0 && x+imx < adapt->getWidth() && x >= 0 && x < w
            && y+imy >= 0 && y+imy < adapt->getHeight() && y >= 0 && y < h)
        {
          double v=adapt->getIntensityOfPixel(x+imx, y+imy);

          if (std::isfinite(v))
          {
            adapt->setMinIntensity(v);
            redrawImage();
            updateInfo();
          }
        }
      }

      break;

    case 'w':
      if (adapt != 0)
      {
        int w, h;

        getSize(w, h);

        if (x+imx >= 0 && x+imx < adapt->getWidth() && x >= 0 && x < w
            && y+imy >= 0 && y+imy < adapt->getHeight() && y >= 0 && y < h)
        {
          double v=adapt->getIntensityOfPixel(x+imx, y+imy);

          if (std::isfinite(v))
          {
            adapt->setMaxIntensity(v);
            redrawImage();
            updateInfo();
          }
        }
      }

      break;

    case 'm':
      if (adapt != 0)
      {
        switch (adapt->getMapping())
        {
          case map_raw:
            adapt->setMapping(map_jet);
            break;

          case map_jet:
            adapt->setMapping(map_rainbow);
            break;

          case map_rainbow:
            adapt->setMapping(map_raw);
            break;
        }

        redrawImage();
      }

      break;

    case 'i':
      showinfo=!showinfo;
      updateInfo();
      break;

    case 'h':
      if (!hasInfoText())
      {
        setInfoText(helptext.c_str());
      }
      else
      {
        setInfoText("");
      }

      break;

    case 'v':
      if (!hasInfoText())
      {
        std::ostringstream out;

        out << "This program is based on cvkit version " << VERSION << "\n";
        out << "Copyright (C) 2016, 2017 Roboception GmbH\n";
        out << "Copyright (C) 2014, 2015 Institute of Robotics and Mechatronics, German Aerospace Center\n";
        out << "Author: Heiko Hirschmueller\n";
        out << "Contact: heiko.hirschmueller@roboception.de\n";
#ifdef INCLUDE_GDAL
        out << "\n";
        out << "This program is based in part on the Geospatial Data Abstraction Library (GDAL).\n";
#endif
#if defined (INCLUDE_GDAL) || defined (INCLUDE_JPEG)
        out << "\n";
        out << "This program is based in part on the work of the Independent JPEG Group.\n";
#endif
#if defined (INCLUDE_PNG)
        out << png_get_copyright(0) << "\n";
#endif
        setInfoText(out.str().c_str());
      }
      else
      {
        setInfoText("");
      }

      break;

    case 's':
      if (adapt != 0)
      {
        setSize(adapt->getWidth(), adapt->getHeight());
      }

      break;

    case 'l':
      if (adapt != 0)
      {
        // rotation around center of image part

        int x=imx;
        int y=imy;
        int w, h;

        getSize(w, h);

        if (adapt->getFlip())
        {
          x=adapt->getWidth()-w-x;
        }

        imx=y+h/2-w/2;
        imy=adapt->getWidth()-1-(x+w/2)-h/2;

        if (adapt->getFlip())
        {
          imx=adapt->getHeight()-w-imx;
        }

        adapt->setRotationFlip(adapt->getRotation()-1, adapt->getFlip());

        redrawImage();
        updateInfo();
      }

      break;

    case 'r':
      if (adapt != 0)
      {
        // rotation around center of image part

        int x=imx;
        int y=imy;
        int w, h;

        getSize(w, h);

        if (adapt->getFlip())
        {
          x=adapt->getWidth()-w-x;
        }

        imx=adapt->getHeight()-1-(y+h/2)-w/2;
        imy=x+w/2-h/2;

        if (adapt->getFlip())
        {
          imx=adapt->getHeight()-w-imx;
        }

        adapt->setRotationFlip(adapt->getRotation()+1, adapt->getFlip());

        redrawImage();
        updateInfo();
      }

      break;

    case 'f':
      if (adapt != 0)
      {
        // flip at center of image part

        int w, h;

        getSize(w, h);

        imx=adapt->getWidth()-w-imx;

        adapt->setRotationFlip(adapt->getRotation(), !adapt->getFlip());

        redrawImage();
        updateInfo();
      }

      break;

    case 'q':
      sendClose();
      break;
  }

  if (key == k_esc)
  {
    sendClose();
  }
}

char ImageWindow::getLastKey()
{
  char ret=lastkey;

  lastkey='\0';

  return ret;
}

}
