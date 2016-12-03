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
#include <cstring>
#include <iostream>
#include <cstdlib>

#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

#include <assert.h>

using std::cerr;
using std::endl;

namespace gutil
{

struct ThreadData
{
    bool running;
    pthread_t thread;

    ThreadFunction *tfct;
    ParallelFunction *pfct;

    long start;
    long end;
    long step;
};

namespace
{

void *ptfct(void *fa)
{
    ThreadData *p=reinterpret_cast<ThreadData *>(fa);

    if (p->tfct != 0)
      p->tfct->run();

    if (p->pfct != 0)
      p->pfct->run(p->start, p->end, p->step);

    return 0;
}

}

Thread::Thread()
{
    p=new ThreadData();
    p->running=false;
}

Thread::~Thread()
{
    if (p->running)
    {
      pthread_cancel(p->thread);
      pthread_join(p->thread, 0);
    }

    delete p;
}

void Thread::create(ThreadFunction &fct)
{
    assert(!p->running);

    pthread_attr_t attr;
    pthread_attr_init(&attr);

    p->tfct=&fct;
    p->pfct=0;
    p->start=0;
    p->end=0;
    p->step=0;

#ifndef NDEBUG
    int err=
#endif
    pthread_create(&(p->thread), &attr, ptfct, p);

    pthread_attr_destroy(&attr);

    assert(err == 0);

    p->running=true;
}

void Thread::create(ParallelFunction &fct, long start, long end, long step,
  int affinity)
{
    assert(!p->running);

    pthread_attr_t attr;
    pthread_attr_init(&attr);

#ifdef _GNU_SOURCE
    if (affinity >= 0 && getMaxThreads() > 1 &&
      getMaxThreads() == getProcessingUnits())
    {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(affinity, &cpuset);

      pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
    }
#endif

    p->tfct=0;
    p->pfct=&fct;
    p->start=start;
    p->end=end;
    p->step=step;

#ifndef NDEBUG
    int err=
#endif
    pthread_create(&(p->thread), &attr, ptfct, p);

    pthread_attr_destroy(&attr);

    assert(err == 0);

    p->running=true;
}

void Thread::join()
{
    if (p->running)
    {
#ifndef NDEBUG
      int err=
#endif
      pthread_join(p->thread, 0);

      p->running=false;

      assert(err == 0);
    }
}

namespace
{

int procunits=0;
int nthreads=0;

}

int Thread::getProcessingUnits()
{
    if (procunits <= 0)
    {
      procunits=sysconf(_SC_NPROCESSORS_CONF);

      if (procunits <= 0)
      {
        cerr << "Cannot determine number of CPUs, assuming one!" << endl;
        procunits=1;
      }

      const char *s=std::getenv("CVKIT_MAX_THREADS");

      if (s != 0)
      {
        int n=std::max(1, std::atoi(s));

        if (n < procunits)
          procunits=n;
      }
    }

    return procunits;
}

int Thread::getMaxThreads()
{
    if (nthreads <= 0)
      nthreads=getProcessingUnits();

    return nthreads;
}

void Thread::setMaxThreads(int n)
{
    if (n <= 0 || n > getProcessingUnits())
      n=getProcessingUnits();

    nthreads=n;
}

}