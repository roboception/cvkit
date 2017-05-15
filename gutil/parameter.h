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

#ifndef GUTIL_PARAMETER_H
#define GUTIL_PARAMETER_H

#include "exception.h"

#include <string>
#include <sstream>
#include <vector>
#include <set>
#include <stdexcept>
#include <algorithm>

namespace gutil
{

class Parameter
{
  public:

    /**
     * argc/argv are given in the same format as used for main(), e.g. argv[0]
     * is the program name. If the special parameter @<f> is encountered, it is
     * replaced by a list of parameters that is read from the file f. Each line
     * of the file is interpreted as a parameter in this case.
     *
     * pdef is a 0-terminated list of c-strings with a description of
     * parameters and arguments. The description is also used for checking the
     * validity of parameters. Format, line by line:
     *
     * -<param> Definition of a parameter
     *  <arg>   Argunment that belongs to the previous parameter
     * <arg>    Free argument, i.e. without a parameter
     *          Empty means that this is only description
     *
     * Everything after the first '#' character in each line is used as
     * description.
     */

    Parameter(int argc, char *argv[], const char *pdef[], const char *creator=0);

    /**
     * Add parameter definitions and help.
     */

    void addParamDef(const char *pdef[]);

    /**
     * Print help to output stream.
     */

    void printHelp(std::ostream &out, size_t columns=80) const;

    /**
     * Returns the number of remaining values (i.e. parameters or argument).
     */

    int remaining() const
    {
      return static_cast<int>(list.size()-pos);
    }

    /**
     * Reduces the pointer to the current value so that it points to the
     * previous value.
     */

    void previous()
    {
      pos=std::max(static_cast<size_t>(0), pos-1);
    }

    /**
     * Returns true, if there is another value and this value is a valid
     * parameter.
     */

    bool isNextParameter();

    /**
     * Returns the next parameter. Throws an exception if the next value does
     * not start with '-' or if it is not in the definition list 'pdef'.
     */

    void nextParameter(std::string &p);

    /**
     * Searches the given parameter and sets the read position to the value
     * behind, for reading the arguments. If an empty std::string is given, then the
     * read position is set to the beginning. Returns false, if the parameter is
     * not given. The parameter must be in the definition list 'pdef'.
     */

    bool findParameter(const std::string &p);

    /**
     * Returns the next argument as std::string. If opt is not empty, it is
     * interpreted as a list of possible values, separated by '|'. An exception
     * is thrown if the next value is a parameter (i.e. start with '-') or if
     * the value is not in the list of possible values.
     */

    void nextString(std::string &s, const char *opt=0);

    /**
     * Returns the next argument in the provided variable. An exception is
     * thrown if the next value is a parameter (i.e. starts with '-') or if the
     * value cannot be converted.
     */

    template <class T> void nextValue(T &v)
    {
      std::string s;

      nextString(s);

      std::istringstream in(s);
      in >> v;

      if (in.fail())
      {
        throw IOException("Format error of argument: "+s);
      }
    }

  private:

    std::vector<std::string> list;
    std::vector<std::string> def;
    std::string              creat;
    std::set<std::string>    allparam;
    size_t                   cparam;
    size_t                   pos;
};

}

#endif
