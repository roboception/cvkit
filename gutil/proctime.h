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

#ifndef GUTIL_PROCTIME_H
#define GUTIL_PROCTIME_H

#ifdef WIN32
#include <Windows.h>
#undef min
#undef max
#else
#include <time.h>
#endif

namespace gutil
{

#ifdef WIN32

class ProcTime
{
  private:

    ULONGLONG tstart;
    double total;

  public:

    ProcTime()
    {
      tstart=0;
      total=0;
    }

    void start()
    {
      tstart=GetTickCount64();
    }

    void stop()
    {
      ULONGLONG tend=GetTickCount64();
      total+=(tend-tstart)/1000.0;
      tstart=tend;
    }

    void clear()
    {
      total=0;
    }

    double elapsed()
    {
      return total;
    }

    static double monotonic()
    {
      return GetTickCount64()/1000.0;
    }

    static double current()
    {
      return GetTickCount64()/1000.0;
    }
};

#else

class ProcTime
{
  private:

    struct timespec tstart;
    double total;

  public:

    ProcTime()
    {
      tstart.tv_sec=0;
      tstart.tv_nsec=0;
      total=0;
    }

    void start()
    {
      clock_gettime(CLOCK_MONOTONIC, &tstart);
    }

    void stop()
    {
      struct timespec tend;

      clock_gettime(CLOCK_MONOTONIC, &tend);

      total+=tend.tv_sec-tstart.tv_sec+(tend.tv_nsec-tstart.tv_nsec)/1000000000.0;
      tstart=tend;
    }

    void clear()
    {
      total=0;
    }

    double elapsed()
    {
      return total;
    }

    static double monotonic()
    {
      struct timespec t;

      clock_gettime(CLOCK_MONOTONIC, &t);

      return t.tv_sec+t.tv_nsec/1000000000.0;
    }

    static double current()
    {
      struct timespec t;

      clock_gettime(CLOCK_REALTIME, &t);

      return t.tv_sec+t.tv_nsec/1000000000.0;
    }
};

#endif

}

#endif
