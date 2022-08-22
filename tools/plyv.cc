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

#include <gvr/model.h>
#include <gvr/cameracollection.h>
#include <gvr/glmain.h>
#include <gvr/glworld.h>
#include <gmath/svector.h>
#include <gmath/camera.h>
#include <gmath/linalg.h>
#include <gutil/parameter.h>
#include <gutil/exception.h>
#include <gutil/version.h>
#include <gutil/misc.h>

#include <cstdlib>

int main(int argc, char *argv[])
{
  try
  {
    gvr::GLInit(argc, argv);

    const char *def[]=
    {
      "# plyv [<options>] <file> ...",
      "#",
      "# The files may be given in ply format or as depth file. In case of a depth file, optional parameters may be appended to the file name, separated by commas. Parameters are:",
      "#",
      "# p=<parameter file>",
      "# i=<image file>",
      "# ds=<s>",
      "# x=<x>,y=<y>,w=<w>,h=<h>",
      "# s=<max step>",
      "#",
      "# Options are:",
      "#",
      "-help # Print help and exit.",
      "-version # Print version and exit.",
      "-spath # Search path for finding parameter files or images that are associated with the depth image. Default is the content of the environment variable CVKIT_SPATH.",
      " <dir list> # List of directories.",
      "-bg # Setting background color.",
      " <r>,<g>,<b> # Background color.",
      "-size # Set size of virtual camera image.",
      " <w> <h> # Size of image. Default: 800 600",
      "-hfov # Horizontal field of view.",
      " <hfov> # Horizontal field of view in degree. Default: 50",
      "-key # Sends the given keycodes to the viewer on startup.",
      " <codes> # Key codes to be set in the given order.",
      0
    };

    gutil::Parameter param(argc, argv, def);

    // handle options

    std::string spath;
    std::string bg;
    std::string keycodes;
    int width=800;
    int height=600;
    double hfov=-1;

    if (getenv("CVKIT_SPATH") != 0)
    {
      spath=getenv("CVKIT_SPATH");
    }

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

      if (p == "-spath")
      {
        param.nextString(spath);
      }

      if (p == "-bg")
      {
        param.nextString(bg);
      }

      if (p == "-key")
      {
        param.nextString(keycodes);
      }

      if (p == "-size")
      {
        param.nextValue(width);
        param.nextValue(height);
      }

      if (p == "-hfov")
      {
        param.nextValue(hfov);
        hfov=hfov/180*gmath::pi;
      }
    }

    if (param.remaining() < 1)
    {
      gutil::showError("No PLY files given");
      param.printHelp(std::cout);
      return 10;
    }

    // initialization

    gvr::GLInitWindow(-1, -1, width, height, "plyv");

    gvr::GLWorld          world(width, height, hfov, keycodes);
    gvr::CameraCollection camlist;

    if (bg.size() > 0)
    {
      std::vector<std::string> list;

      gutil::split(list, bg, ',');

      if (list.size() != 3)
      {
        throw gutil::InvalidArgumentException(std::string("Illegal format: ")+bg);
      }

      float r=std::max(0.0f, std::min(1.0f, std::stoi(list[0])/255.0f));
      float g=std::max(0.0f, std::min(1.0f, std::stoi(list[1])/255.0f));
      float b=std::max(0.0f, std::min(1.0f, std::stoi(list[2])/255.0f));

      world.setBackgroundColor(r, g, b);
    }

    // add models

    bool first=true;
    int id=gvr::ID_MODEL_START;
    gmath::Vector3d offset;

    while (param.remaining() > 0)
    {
      std::string file;
      param.nextString(file);

      // splitting filename into directory and name

      size_t i=file.find_last_of("/\\");

      std::string dir, name;

      if (i != file.npos)
      {
        dir=file.substr(0, i+1);
        name=file.substr(i+1);
      }
      else
      {
        name=file;
      }

      // derive basepath and offset from first model

      if (first)
      {
        world.setCapturePrefix((dir+"capture").c_str());
      }

      if (file.size() > 4 && file.find(',') == std::string::npos &&
          (file.compare(file.size()-4, 4, ".txt") == 0 ||
           file.compare(file.size()-4, 4, ".TXT") == 0))
      {
        camlist.loadCamera(file.c_str());
        world.showCameras(true);
      }
      else
      {
        // load model

        gvr::Model *model=gvr::loadModel(file.c_str(), spath.c_str(), true);

        if (model != 0)
        {
          if (first)
          {
            world.setOffset(model->getOrigin());
            first=false;
          }

          // set model id

          model->setID(id++);

          // add model to world

          model->translate(-world.getOffset());
          world.addModel(*model);

          delete model;

          // load Middlebury cameras automatically

          if (name.size() >= 9 && name.compare(0, 4, "disp") == 0)
          {
            try
            {
              camlist.loadCamera((dir+"calib.txt").c_str());
            }
            catch (const gutil::IOException &)
            { }
          }
        }
      }
    }

    // add camera models

    if (camlist.getCameraCount() > 0)
    {
      camlist.translate(-world.getOffset());
      world.addModel(camlist);
    }

    world.resetCamera();

    // enter main loop

    GLMainLoop(world);
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
