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

#include "misc.h"

#include <sstream>
#include <iostream>
#include <assert.h>

#ifdef __GNUC__
#include <dirent.h>
#elif defined(WIN32)
#include <Windows.h>
#undef min
#undef max
#endif

bool gutil::isMSBFirst()
{
  int p=1;
  char *c=reinterpret_cast<char *>(&p);

  if (c[0] == 1)
  {
    return false;
  }

  return true;
}

void gutil::trim(std::string &s)
{
  size_t pos;

  pos=0;

  while (pos < s.size() && isspace(s[pos]))
  {
    pos++;
  }

  if (pos > 0)
  {
    s=s.substr(pos);
  }

  pos=s.size();

  while (pos > 0 && isspace(s[pos-1]))
  {
    pos--;
  }

  if (pos < s.size())
  {
    s=s.substr(0, pos);
  }
}

void gutil::split(std::vector<std::string> &list, const std::string &s, char delim, bool skip_empty)
{
  std::stringstream in(s);
  std::string elem;

  while (getline(in, elem, delim))
  {
    if (!skip_empty || elem.size() > 0)
    {
      list.push_back(elem);
    }
  }
}

void gutil::getFileList(std::set<std::string> &list, const std::string &prefix,
                        const std::string &suffix)
{
#ifdef __GNUC__
  std::string dir="", pref=prefix;
  size_t pos=pref.find_last_of("/\\");

  if (pos < pref.size())
  {
    dir=pref.substr(0, pos+1);
    pref=pref.substr(pos+1);
  }

  DIR *p;

  if (dir.size() > 0)
  {
    p=opendir(dir.c_str());
  }
  else
  {
    p=opendir(".");
  }

  if (p == 0)
  {
    throw IOException("Cannot open directory: "+dir);
  }

  list.clear();

  struct dirent *entry=readdir(p);

  while (entry != 0)
  {
    std::string name=entry->d_name;

    if (name.size() >= pref.size()+suffix.size() &&
        name.find(pref) ==  0 && (suffix.size() == 0 ||
                                  name.rfind(suffix) == name.size()-suffix.size()))
    {
      list.insert(dir+name);
    }

    entry=readdir(p);
  }

  closedir(p);
#elif defined(WIN32)
  std::string dir="";
  size_t pos=prefix.find_last_of("/\\");

  if (pos < prefix.size())
  {
    dir=prefix.substr(0, pos+1);
  }

  HANDLE p;
  WIN32_FIND_DATA data;

  list.clear();

  p=FindFirstFileA((prefix+"*"+suffix).c_str(), &data);

  if (p != INVALID_HANDLE_VALUE)
  {
    do
    {
      list.insert(dir+data.cFileName);
    }
    while (FindNextFileA(p, &data));

    FindClose(p);
  }

#else
  assert(false);
#endif
}

bool gutil::skip(std::istream &in, const char *expected)
{
  if (in.good())
  {
    std::string s;
    in >> s;

    if (s == expected)
    {
      return true;
    }

#ifndef NDEBUG
    std::cerr << "gutil::skip() expected '" << expected << "' but got '" << s << "'" << std::endl;
#endif

    in.setstate(std::ios_base::failbit);
  }

  return false;
}
