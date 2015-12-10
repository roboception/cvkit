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

#ifndef GUTIL_THREAD_H
#define GUTIL_THREAD_H

#include <iostream>

namespace gutil
{

/**
  This class defines a function that can be called by executed in a thread.
*/

class ThreadFunction
{
  public:
    
    /**
      Function that can be called by a thread.
      
      @param start Start of iteration.
      @param end   End of iteration. It can be smaller than start.
      @param step  Increment of each iteration. It can also be negative.
    */
    
    virtual void run(long start, long end, long step)=0;
};

struct ThreadData;

class Thread
{
  public:
    
    Thread();
    ~Thread();
    
    /*
     * Runs the given function in an own thread, with the given arguments. If
     * affinity is >= 0 (and getMaxThreads() > 1 and getMaxThreads() ==
     * getProcessingUnits()) then the thread will only run on exactly this
     * processing unit. affinity must be between 0 and getProcessingUnits()-1).
     */
    
    void create(ThreadFunction &fct, long start=0, long end=0, long step=1,
      int affinity=-1);
    
    /*
     * Wait until the thread has finished.
     */
    
    void join();
  
    /*
     * Returns the number of processing units. If it cannot be determined,
     * then 1 is returned.
     */
    
    static int getProcessingUnits();
    
    /*
     * Get/set the maximum number of threads that should be started in
     * parallel.
     */
    
    static int getMaxThreads();
    static void setMaxThreads(int n);
    
  private:
    
    ThreadData *p;
};

/*
 * Runs the maximum number of threads for interleaved processing of the given
 * function, from start until end (including end). The function waits until all
 * threads have finished.
 */

inline void runParallel(ThreadFunction &fct, long start, long end, long step)
{
    int n=Thread::getMaxThreads();
    n=std::min(n, static_cast<int>((end-start+1+step-1)/step));
    
    if (n > 1)
    {
      Thread *thread=new Thread [n];
      
      for (int i=0; i<n; i++)
        thread[i].create(fct, start+i*step, end, n*step, -1);
      
      for (int i=0; i<n; i++)
        thread[i].join();
      
      delete [] thread;
    }
    else
      fct.run(start, end, step);
}

}

#endif
