/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2016 Roboception GmbH
 * Copyright (c) 2014 Institute of Robotics and Mechatronics, German Aerospace Center
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

#include "thread.h"

#include <gutil/exception.h>

#include <iostream>
#include <thread>
#include <assert.h>
#include <cstdlib>
#include <algorithm>

namespace
{
int procunits=0;
int nthreads=0;
}

namespace gutil
{

struct ThreadData {};

Thread::Thread()
{
  p=0;
}

Thread::~Thread()
{
  delete reinterpret_cast<std::thread *>(p);
}

void Thread::create(ThreadFunction &fct)
{
  assert(p == 0);

  p=reinterpret_cast<ThreadData *>(new std::thread(&ThreadFunction::run, &fct));
}

void Thread::create(ParallelFunction &fct, long start, long end, long step,
                    int affinity)
{
  assert(p == 0);

  p=reinterpret_cast<ThreadData *>(new std::thread(&ParallelFunction::run, &fct,
                                   start, end, step));
}

void Thread::join()
{
  if (p != 0)
  {
    reinterpret_cast<std::thread *>(p)->join();
    p=0;
  }
}

int Thread::getProcessingUnits()
{
  if (procunits <= 0)
  {
    procunits=std::thread::hardware_concurrency();

    if (procunits <= 0)
    {
      std::cerr << "Cannot determine number of CPUs, assuming one!" << std::endl;
      procunits=1;
    }

    const char *s=std::getenv("CVKIT_MAX_THREADS");

    if (s != 0)
    {
      int n=std::max(1, std::atoi(s));

      if (n < procunits)
      {
        procunits=n;
      }
    }
  }

  return procunits;
}

int Thread::getMaxThreads()
{
  if (nthreads <= 0)
  {
    nthreads=getProcessingUnits();
  }

  return nthreads;
}

void Thread::setMaxThreads(int n)
{
  if (n <= 0 || n > getProcessingUnits())
  {
    n=getProcessingUnits();
  }

  nthreads=n;
}

}
