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

#ifndef GUTIL_PROPERTIES_H
#define GUTIL_PROPERTIES_H

#include "exception.h"

#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <stdexcept>

#include <iostream>

namespace gutil
{

/**
 * The properties class stores key/value pairs. The keys are always std::strings,
 * while the values can be native types or objects for which the istream >> or
 * ostream << operators are overloaded.
 */

class Properties
{
  public:

    Properties() { }

    ///Loads the named file
    ///Throws IOException
    explicit Properties(const char *name)
    {
      load(name);
    }

    bool contains(const char *key) const
    {
      return data.find(key) != data.end();
    }

    bool operator == (const Properties &p) const;

    bool operator != (const Properties &p) const
    {
      return !(operator == (p));
    }

    void getString(const char *key, std::string &value, const char *defvalue=0)
    const;

    template <class T> void getValue(const char *key, T &value,
                                     const char *defvalue=0) const
    {
      std::string v;

      getString(key, v, defvalue);

      std::istringstream in(v);
      in >> value;

      if (in.fail())
      {
        throw IOException("Format error: "+std::string(key)+"="+v);
      }
    }

    template <class T> void getValue(const char *key, T &value,
                                     const T &defvalue) const
    {
      std::string v;

      getString(key, v, "");

      value=defvalue;

      if (v.size() > 0)
      {
        std::istringstream in(v);
        in >> value;

        if (in.fail())
        {
          throw IOException("Format error: "+std::string(key)+"="+v);
        }
      }
    }

    void getStringVector(const char *key, std::vector<std::string> &value,
                         const char *defvalue=0, const char sep='|') const;

    template <class T> void getValueVector(const char *key, std::vector<T> &value,
                                           const char *defvalue=0, const char sep='|') const
    {
      std::vector<std::string> vs;

      getStringVector(key, vs, defvalue, sep);

      value.clear();

      for (size_t i=0; i<vs.size(); i++)
      {
        T v;

        std::istringstream in(vs[i]);
        in >> v;

        if (in.fail())
        {
          throw IOException("Format error: "+std::string(key)+"="+vs[i]);
        }

        value.push_back(v);
      }
    }

    void putString(const char *key, const std::string &value);

    template <class T> void putValue(const char *key, const T &value)
    {
      std::ostringstream out;
      out << value;
      putString(key, out.str());
    }

    template <class T> void putValue(const char *key, const T &value, int precision)
    {
      std::ostringstream out;
      out.precision(precision);
      out << value;
      putString(key, out.str());
    }

    template <class T> void putValueVector(const char *key,
                                           const std::vector<T> &value, const char sep='|')
    {
      std::ostringstream out;
      typename std::vector<T>::const_iterator it;

      for (it=value.begin(); it+1<value.end(); ++it)
      {
        out << *it << sep;
      }

      if (it < value.end())
      {
        out << *it;
      }

      putString(key, out.str());
    }

    void putStringVector(const char *key, const std::vector<std::string> &value,
                         const char sep='|')
    {
      putValueVector(key, value, sep);
    }

    void remove(const std::string &key)
    {
      data.erase(key);
    }

    void clear()
    {
      data.clear();
    }

    bool isEmpty() const
    {
      return data.empty();
    }

    ///Throws IOException
    void load(const char *name);
    void load(std::istream& in);
    void save(const char *name, const char *comment=0) const;
    void save(std::ostream& out, const char *comment=0) const;
    void print();

  private:
    void load(std::istream& in, const char *name);

  private:

    std::map<std::string, std::string> data;
};

}

#endif
