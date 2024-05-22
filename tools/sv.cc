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

#include "gutil/parameter.h"
#include "bgui/fileimagewindow.h"
#include "gutil/exception.h"
#include "gutil/version.h"
#include "gutil/misc.h"
#include "bgui/messagewindow.h"

#include <string>
#include <set>
#include <fstream>

int main(int argc, char *argv[])
{
  try
  {
    int x=-1, y=-1;
    int w=-1, h=-1;
    bool size_max=false;
    double scale=0;
    double imin=0;
    double imax=0;
    double vmin=-std::numeric_limits<float>::max();
    double vmax=std::numeric_limits<float>::max();
    bgui::FileImageWindow::keep kp=bgui::FileImageWindow::keep_none;
    bgui::mapping map=bgui::map_raw;
    int channel=-1;
    bool watch=false;
    std::vector<std::string> list;
    int first=0;

    const char *def[]=
    {
      "# sv [<options>] <image file> ...",
      "#",
      "-help # Print help and exit.",
      "-version # Print version and exit.",
      "-pos # Set initial position of window.",
      " <x> <y> # Position of window.",
      "-size # Set initial size of window. It will be limited by the screen size.",
      " <w> <h> # Width and height of window.",
      "-maxsize # Set initial maximum size of the window. It can be smaller, depending on the first image.",
      " <w> <h> # Width and height of window.",
      "-scale # Set initial scale factor (implies -keep).",
      " <s> # Initial scale factor.",
      "-select # Select a color channel.",
      " R|G|B # Color channel.",
      "-imin # Set initial minimum intensity (implies -keepall).",
      " <v> # Intensity.",
      "-imax # Set initial maximum intensity (implies -keepall).",
      " <v> # Intensity.",
      "-vmin # Set minimum valid intensity.",
      " <v> # Intensity.",
      "-vmax # Set maximum valid intensity.",
      " <v> # Intensity.",
      "-keep # Keep settings, except intensity range, when switching between images.",
      "-keepall # Keep all settings when switching between images.",
      "-watch # Watches the current image file for changes and reloads automatically.",
      "-map # Mapping for greyscale images: raw (default), jet, rainbow.",
      0
    };

    gutil::Parameter param(argc, argv, def);

    // handle options

    while (param.isNextParameter())
    {
      std::string p;

      param.nextParameter(p);

      if (p == "-help")
      {
        param.printHelp(std::cout);
        return 0;
      }

      if (p == "-version")
      {
        std::cout << "This program is part of cvkit version " << VERSION << std::endl;
        return 0;
      }

      if (p == "-pos")
      {
        param.nextValue(x);
        param.nextValue(y);
      }

      if (p == "-size")
      {
        param.nextValue(w);
        param.nextValue(h);
        size_max=false;
      }

      if (p == "-maxsize")
      {
        param.nextValue(w);
        param.nextValue(h);
        size_max=true;
      }

      if (p == "-scale")
      {
        param.nextValue(scale);

        if (scale > 0)
        {
          if (kp == bgui::FileImageWindow::keep_none)
          {
            kp=bgui::FileImageWindow::keep_most;
          }
        }
        else
        {
          scale=0;
        }
      }

      if (p == "-select")
      {
        std::string s;

        param.nextString(s, "R|G|B");

        if (s == "R")
        {
          channel=0;
        }
        else if (s == "G")
        {
          channel=1;
        }
        else if (s == "B")
        {
          channel=2;
        }
      }

      if (p == "-imin")
      {
        param.nextValue(imin);
        kp=bgui::FileImageWindow::keep_all;
      }

      if (p == "-imax")
      {
        param.nextValue(imax);
        kp=bgui::FileImageWindow::keep_all;
      }

      if (p == "-vmin")
      {
        param.nextValue(vmin);
      }

      if (p == "-vmax")
      {
        param.nextValue(vmax);
      }

      if (p == "-keep")
      {
        kp=bgui::FileImageWindow::keep_most;
      }

      if (p == "-keepall")
      {
        kp=bgui::FileImageWindow::keep_all;
      }

      if (p == "-watch")
      {
        watch=true;
      }

      if (p == "-map")
      {
        std::string s;

        param.nextString(s, "raw|jet|rainbow");

        if (s == "jet")
        {
          map=bgui::map_jet;
        }

        if (s == "rainbow")
        {
          map=bgui::map_rainbow;
        }
      }
    }

    // collect files

    if (param.remaining() < 1)
    {
      bgui::MessageWindow win("Error", "No image files give on the command line", 400, 100);
      param.printHelp(std::cout);
      return 10;
    }

    while (param.remaining() > 0)
    {
      std::string file;
      param.nextString(file);
      list.push_back(file);
    }

    // if exactly one file is given, get all files of that directory for
    // convenience

    if (list.size() == 1)
    {
      std::ifstream in;
      in.open(list[0].c_str(), std::ifstream::in);
      in.close();

      if (!in.fail())
      {
        std::string name=list[0];
        std::string dir="";

        size_t i=name.find_last_of("\\/");

        if (i != name.npos)
        {
          dir=name.substr(0, i+1);
        }

        try
        {
          std::set<std::string> content;
          gutil::getFileList(content, dir, "");

          list.clear();

          for (std::set<std::string>::iterator it=content.begin(); it != content.end(); ++it)
          {
            list.push_back(*it);
          }

          sort(list.begin(), list.end());

          first=-1;

          for (size_t k=0; k<list.size(); k++)
          {
            if (list[k] == name)
            {
              first=static_cast<int>(k);
            }
          }

          if (first == -1)
          {
            list.insert(list.begin(), name);
            first=0;
          }
        }
        catch (const std::exception &)
        { }
      }
    }

    // open windows and show first image

    std::string viewcmd=argv[0];

    size_t pos=viewcmd.find_last_of("/\\");

    if (pos != viewcmd.npos)
    {
      viewcmd=viewcmd.substr(0, pos+1);
    }
    else
    {
      viewcmd.clear();
    }

#ifdef WIN32
    viewcmd=viewcmd+"plyv.exe";
#else
    viewcmd=viewcmd+"plyv";
#endif

    bgui::FileImageWindow win(list, first, watch, x, y, w, h, size_max, scale,
                              imin, imax, vmin, vmax, kp, map, channel, viewcmd.c_str());

    win.waitForClose();
  }
  catch (gutil::Exception &ex)
  {
    ex.print();
  }
  catch (...)
  {
    gutil::showError("An unknown exception occured");
  }

  return 0;
}
