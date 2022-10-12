/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2022, Roboception GmbH
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

#include <gutil/parameter.h>
#include <gutil/properties.h>

#include <iostream>
#include <fstream>

int main(int argc, char *argv[])
{
  try
  {
    const char *def[]=
    {
      "# prop2octave <property-file> [<output-file>]",
      "#",
      "# Converts a property file (i.e. key=value) into octave text format.",
      "#",
      "-help # Print help and exit.",
      0
    };

    gutil::Parameter param(argc, argv, def);

    // handle options

    std::string inname;
    std::string outname;

    while (param.isNextParameter())
    {
      std::string p;

      param.nextParameter(p);

      if (p == "-help")
      {
        param.printHelp(std::cout);
        return 0;
      }
    }

    if (param.remaining() < 1)
    {
      param.printHelp(std::cout);
      return 10;
    }

    param.nextString(inname);

    if (param.remaining() > 0)
    {
      param.nextString(outname);
    }

    // load properties file

    gutil::Properties prop(inname.c_str());

    if (outname.size() > 0)
    {
      std::ofstream out;

      out.exceptions(std::ios_base::failbit | std::ios_base::badbit | std::ios_base::eofbit);
      out.open(outname);

      prop.saveOctave(out, ("Converted from "+inname).c_str());

      out.close();
    }
    else
    {
      prop.saveOctave(std::cout, ("Converted from "+inname).c_str());
    }
  }
  catch (const std::exception &ex)
  {
    std::cerr << ex.what() << std::endl;
  }
}
