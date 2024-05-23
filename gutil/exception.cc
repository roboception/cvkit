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

#include "exception.h"

#if defined(DEBUG) && defined(__GNUC__)
#include <execinfo.h>
#include <sstream>
#endif

#include <cstdlib>
#include <iostream>

namespace gutil
{

Exception::Exception(const std::string &type, const std::string &message): s(type+": "+message)
{
#if defined(DEBUG) && defined(__GNUC__)
  std::ostringstream os;
  void *ptr[64];
  int nptr=backtrace(ptr, 64);
  char **sym=backtrace_symbols(ptr, nptr);

  for (int i=1; i<nptr-1; i++)
  {
    os << sym[i] << "\n";
  }

  if (nptr > 1)
  {
    os << sym[nptr-1];
  }

  free(sym);

  bt=os.str();
#endif
}

void Exception::print() const
{
  std::cerr << s << std::endl;

#if defined(DEBUG) && defined(__GNUC__)
  size_t s0=0;

  while (s0 != std::string::npos)
  {
    if (bt[s0] == '\n')
    {
      s0++;
    }

    size_t e0=bt.find_first_of(" \t(", s0);
    size_t s1=bt.find_first_of("[", e0);
    size_t e1=bt.find_first_of("]", s1);

    if (e0 != std::string::npos && s1 != std::string::npos &&
        e1 != std::string::npos)
    {
      std::string cmd="addr2line -e "+bt.substr(s0, e0-s0)+" "+
                      bt.substr(s1+1, e1-s1-1);

      if (system(cmd.c_str()) == -1)
      {
        std::cerr << cmd << " failed!" << std::endl;
      }
    }

    s0=bt.find_first_of("\n", e1);
  }

#endif
}

}
