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
#include <gvr/ply.h>
#include <gmath/svector.h>
#include <gutil/parameter.h>
#include <gutil/version.h>

#include <cstdlib>

int main(int argc, char *argv[])
{
  const char *def[]=
  {
    "# plycmd <file> [<options>]",
    "#",
    "# The input file may be given in ply format or as depth file. In case of a depth file, optional parameters may be appended to the file name, separated by commas. Parameters are:",
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
    "-header # Print header of ply file.",
    "-reduce # Removes all data that is unnecessary for visualization.",
    "-ascii # Store ply as ascii.",
    " <file> # Output file.",
    "-out # Store ply as binary.",
    " <file> # Output file.",
    0
  };

  gutil::Parameter param(argc, argv, def);

  if (param.remaining() < 1)
  {
    param.printHelp(std::cout);
    return 10;
  }

  // handle options

  std::string name;
  std::string spath;
  bool   all=true;
  std::string aout;
  std::string bout;

  if (getenv("CVKIT_SPATH") != 0)
  {
    spath=getenv("CVKIT_SPATH");
  }

  if (!param.isNextParameter())
  {
    param.nextString(name);
  }

  while (param.remaining() > 0)
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

    if (p == "-header")
    {
      gvr::PLYReader ply;

      if (ply.open(name.c_str()))
      {
        ply.printHeader();
      }
      else
      {
        std::cerr << "Not a ply file: " << name << std::endl;
      }
    }

    if (p == "-reduce")
    {
      all=false;
    }

    if (p == "-ascii")
    {
      param.nextString(aout);
    }

    if (p == "-out")
    {
      param.nextString(bout);
    }
  }

  if (aout.size() > 0 || bout.size() > 0)
  {
    if (name.size() > 0)
    {
      gvr::Model *model=gvr::loadModel(name.c_str(), spath.c_str(), true);

      if (aout.size() > 0)
      {
        model->savePLY(aout.c_str(), all, gvr::ply_ascii);
      }

      if (bout.size() > 0)
      {
        model->savePLY(bout.c_str(), all, gvr::ply_binary);
      }

      delete model;
    }
    else
    {
      std::cerr << "No input file given!" << std::endl;
      return 10;
    }
  }
}