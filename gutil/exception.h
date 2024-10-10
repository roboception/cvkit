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

#ifndef GUTIL_EXCEPTION_H
#define GUTIL_EXCEPTION_H

#include <string>
#include <exception>

namespace gutil
{

class Exception : public std::exception
{
  private:

    std::string s;
#if defined(DEBUG) && defined(__GNUC__)
    std::string bt;
#endif

  public:

    Exception(const std::string &type, const std::string &message);

    Exception(const Exception &e)
    {
      s=e.s;
#if defined(DEBUG) && defined(__GNUC__)
      bt=e.bt;
#endif
    }

    virtual ~Exception() throw() { }

    virtual const char *what() const throw()
    {
      return s.c_str();
    }

    virtual const char *where() const
    {
#if defined(DEBUG) && defined(__GNUC__)
      return bt.c_str();
#else
      return "";
#endif
    }

    void print() const;
};

class InvalidArgumentException : public Exception
{
  public:

    InvalidArgumentException(const std::string &message) :
      Exception("Invalid argument", message) { }
};

class IOException : public Exception
{
  public:

    IOException(const std::string &message) :
      Exception("IO exception", message) { }
};

}

#endif
