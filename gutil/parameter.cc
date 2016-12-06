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

#include "parameter.h"
#include "misc.h"
#include "version.h"

#include <fstream>
#include <algorithm>

#include <assert.h>

namespace gutil
{

namespace
{

std::string getValue(const std::string &s)
{
  std::string ret=s;
  size_t pos=ret.find('#');

  if (pos < ret.size())
  {
    ret=ret.substr(0, pos);
  }

  trim(ret);

  return ret;
}

std::string getDescription(const std::string &s)
{
  std::string ret;
  size_t pos=s.find('#');

  if (pos < s.size())
  {
    ret=s.substr(pos+1);
  }

  trim(ret);

  return ret;
}

void printDescription(std::ostream &out, std::string descr, size_t col, size_t n)
{
  size_t i=0;
  size_t c=col;

  while (i < descr.size())
  {
    while (i<descr.size() && isspace(descr[i]))
    {
      i++;
    }

    size_t k=i;

    while (k < descr.size() && !isspace(descr[k]))
    {
      k++;
    }

    if (c+1+k-i > n && c > col)
    {
      out << std::endl;

      for (c=0; c<col; c++)
      {
        out << ' ';
      }
    }

    if (c+1+k-i > n)
    {
      k=i+n-c-1;
    }

    if (c > 0)
    {
      out << ' ';
      c++;
    }

    while (i < k)
    {
      out << descr[i++];
      c++;
    }
  }

  out << std::endl;
}

}

Parameter::Parameter(int argc, char *argv[], const char *pdef[],
                     const char *creator)
{
  for (int i=1; i<argc; i++)
  {
    if (argv[i][0] == '@')
    {
      std::ifstream in;
      std::string s;

      in.exceptions(std::ios_base::badbit);
      in.open(argv[i]+1);

      if (!in.good())
      {
        throw IOException(argv[i]+1);
      }

      while (in.good())
      {
        getline(in, s);

        trim(s);

        if (s.size() > 0)
        {
          list.push_back(s);
        }
      }

      in.close();
    }
    else
    {
      list.push_back(std::string(argv[i]));
    }
  }

  for (int i=0; pdef[i] != 0; i++)
  {
    std::string s=pdef[i];
    def.push_back(s);

    s=getValue(s);

    if (s.size() > 0 && s[0] == '-')
    {
      allparam.insert(s);
    }
  }

  if (creator != 0)
  {
    creat=creator;
  }

  cparam=-1;
  pos=0;
}

void Parameter::addParamDef(const char *pdef[])
{
  for (int i=0; pdef[i] != 0; i++)
  {
    std::string s=pdef[i];
    def.push_back(s);

    s=getValue(s);

    if (s.size() > 0 && s[0] == '-')
    {
      allparam.insert(s);
    }
  }
}

void Parameter::printHelp(std::ostream &out, size_t columns)
const
{
  size_t col1=0;

  if (creat.size() > 0)
  {
    out << creat << std::endl;
  }

  for (size_t i=0; i<def.size(); i++)
  {
    std::string val=getValue(def[i]);
    size_t n=val.size();

    if (n > 0 && val[0] == '<' && def[i][0] == ' ')
    {
      n++;
    }

    col1=std::max(col1, n);
  }

  col1++;

  if (col1 > columns/2)
  {
    col1=columns/2;
  }

  for (size_t i=0; i<def.size(); i++)
  {
    std::string val=getValue(def[i]);
    std::string descr=getDescription(def[i]);

    if (val.size() > 0)
    {
      size_t n=0;

      if (val[0] != '-' && def[i][0] == ' ')
      {
        out << "  ";
        n+=2;
      }

      out << val;
      n+=val.length();

      while (n < col1)
      {
        out << ' ';
        n++;
      }

      if (n > col1)
      {
        out << std::endl;
        n=0;
      }

      printDescription(out, descr, col1, columns);
    }
    else
    {
      printDescription(out, descr, 0, columns);
    }
  }
}

bool Parameter::isNextParameter()
{
  bool ret=false;

  if (pos < list.size())
    if (list[pos][0] == '-')
      if (allparam.find(list[pos]) != allparam.end())
      {
        ret=true;
      }

  return ret;
}

void Parameter::nextParameter(std::string &p)
{
  if (pos < list.size())
  {
    if (list[pos][0] == '-')
    {
      if (allparam.find(list[pos]) != allparam.end())
      {
        p=list[pos];
        cparam=pos;
        pos++;
      }
      else
      {
        throw IOException("Undefined parameter: "+list[pos]);
      }
    }
    else
    {
      throw IOException("Parameter expected: "+list[pos]);
    }
  }
  else
  {
    throw IOException("There are no more parameters!");
  }
}

bool Parameter::findParameter(const std::string &p)
{
  assert(allparam.find(p) != allparam.end());

  for (size_t i=0; i<list.size(); i++)
  {
    if (list[i] == p)
    {
      cparam=i;
      pos=i+1;

      return true;
    }
  }

  return false;
}

void Parameter::nextString(std::string &s, const char *opt)
{
  if (pos < list.size())
  {
    if (opt != 0)
    {
      std::string p='|'+std::string(opt)+'|';

      if (p.find('|'+list[pos]+'|') >= p.size())
      {
        throw IOException("Argument '"+list[pos]+"' must be one of: "+opt);
      }
    }

    s=list[pos++];
  }
  else
  {
    throw IOException("There are no more values!");
  }
}

}