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
#include <gutil/parameter.h>
#include <gutil/exception.h>
#include <gutil/version.h>

#include <cstdlib>

using std::cout;
using std::endl;
using std::string;

using gutil::Parameter;
using gutil::showError;
using gutil::IOException;
using gutil::Exception;

using gvr::GLInit;
using gvr::GLInitWindow;
using gvr::GLWorld;
using gvr::CameraCollection;
using gvr::ID_MODEL_START;
using gvr::Model;
using gvr::loadModel;

int main(int argc, char *argv[])
{
    try
    {
      GLInit(argc, argv);
      
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
        0
      };
      
      Parameter param(argc, argv, def);
      
        // handle options
      
      string spath;
      
      if (getenv("CVKIT_SPATH") != 0)
        spath=getenv("CVKIT_SPATH");
      
      while (param.isNextParameter())
      {
        string p;
        
        param.nextParameter(p);
        
        if (p == "-help")
        {
          param.printHelp(cout);
          return 0;
        }
        
        if (p == "-version")
        {
          cout << "This program is part of cvkit version " << VERSION << endl;
          return 0;
        }
        
        if (p == "-spath")
          param.nextString(spath);
      }
      
      if (param.remaining() < 1)
      {
        showError("No PLY files given");
        param.printHelp(cout);
        return 10;
      }
      
        // initialization
      
      GLInitWindow(-1, -1, 800, 600, "plyv");
      
      GLWorld          world(800, 600);
      CameraCollection camlist;
      
        // add models
      
      bool first=true;
      int id=ID_MODEL_START;
      gmath::Vector3d offset;
      
      while (param.remaining() > 0)
      {
        string file;
        param.nextString(file);
        
          // splitting filename into directory and name
        
        size_t i=file.find_last_of("/\\");
        
        string dir, name;
        
        if (i != file.npos)
        {
          dir=file.substr(0, i+1);
          name=file.substr(i+1);
        }
        else
          name=file;
        
          // derive basepath and offset from first model
        
        if (first)
          world.setCapturePrefix((dir+"capture").c_str());
        
        if (file.size() > 4 && file.find(',') == string::npos &&
          (file.compare(file.size()-4, 4, ".txt") == 0 ||
           file.compare(file.size()-4, 4, ".TXT") == 0))
        {
          camlist.loadCamera(file.c_str());
          world.showCameras(true);
        }
        else
        {
            // load model
          
          Model *model=loadModel(file.c_str(), spath.c_str(), true);
          
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
              catch (const IOException &ex)
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
    catch (Exception &ex)
    {
      ex.print();
    }
    catch (...)
    {
      showError("An unknown exception occured");
    }
    
    return 0;
}
