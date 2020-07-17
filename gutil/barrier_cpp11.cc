/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2020 Roboception GmbH
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

#include "barrier.h"
#include "thread.h"

#include <mutex>
#include <condition_variable>

namespace gutil
{

struct BarrierData
{
  int                     generation;
  int                     init_count;
  int                     count;
  std::mutex              mtx;
  std::condition_variable cv;
};

Barrier::Barrier(int c)
{
  p=new BarrierData();

  p->generation=0;

  p->init_count=c;
  if (p->init_count < 1)
  {
    p->init_count=Thread::getMaxThreads();
  }

  p->count=p->init_count;
}

Barrier::~Barrier()
{
  delete p;
}

void Barrier::reinit(int c)
{
  std::unique_lock<std::mutex> lck(p->mtx);

  p->init_count=c;
  if (p->init_count < 1)
  {
    p->init_count=Thread::getMaxThreads();
  }

  p->generation++;
  p->count=p->init_count;
  p->cv.notify_all();
}

int Barrier::getCount()
{
  return p->init_count;
}

void Barrier::wait()
{
  std::unique_lock<std::mutex> lck(p->mtx);
  int gen=p->generation;

  p->count--;
  if (p->count == 0)
  {
    p->generation++;
    p->count=p->init_count;
    p->cv.notify_all();
    return;
  }

  while (gen == p->generation)
  {
    p->cv.wait(lck);
  }
}

}
