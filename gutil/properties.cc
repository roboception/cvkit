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

#include "properties.h"
#include "misc.h"

#include <fstream>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::map;
using std::ifstream;
using std::ofstream;
using std::ios_base;
using std::pair;

namespace gutil
{

void Properties::getString(const char *key, string &value,
  const char *defvalue) const
{
    map<string,string>::const_iterator it=data.find(key);
    
    if (it != data.end())
    {
      value=it->second;
    }
    else if (defvalue != 0)
    {
      value=defvalue;
    }
    else
      throw IOException("Key not found: "+string(key));
}

void Properties::getStringVector(const char *key, vector<string> &value,
  const char *defvalue, const char sep) const
{
    string s;
    size_t start, end;
    
    getString(key, s, defvalue);
    
    value.clear();
    
    start=0;
    end=s.find(sep);
    while (start < s.size())
    {
      value.push_back(s.substr(start, end-start));
      
      start=s.size();
      if (end < s.size())
      {
        start=end+1;
        end=s.find(sep, start);
      }
    }
}

void Properties::putString(const char *key, const string &value)
{
    map<string,string>::iterator it=data.find(key);
    
    if (it != data.end())
      data.erase(key);
    
    data.insert(pair<string,string>(key, value));
}

void Properties::load(const char *name)
{
    ifstream in;
    string   line;
    size_t   pos;
    
    try
    {
      in.exceptions(ios_base::badbit);
      in.open(name);
      
      if (!in.good())
        throw IOException(name);
      
      while (in.good())
      {
        getline(in, line);
        
        trim(line);
        
        pos=line.find('#');
        if (pos != line.npos)
          line=line.substr(0, pos);
        
        if (line.size() > 0)
        {
          pos=line.find('=');
          
          if (pos != line.npos)
          {
            string key=line.substr(0, pos);
            string value=line.substr(pos+1);
            
            trim(key);
            trim(value);
            
            data.insert(pair<string,string>(key, value));
          }
          else
            throw IOException("Format <key>=<value> expected: "+line);
        }
      }
      
      in.close();
    }
    catch (ios_base::failure ex)
    {
      throw IOException(ex.what());
    }
}

void Properties::save(const char *name, const char *comment) const
{
    ofstream out;
    
    try
    {
      out.exceptions(ios_base::failbit | ios_base::badbit | ios_base::eofbit);
      out.open(name);
      
      if (comment != 0)
         out << "# " << comment << endl;
      
      for (map<string,string>::const_iterator it=data.begin(); it!=data.end(); it++)
        out << it->first << "=" << it->second << endl;
      
      out.close();
    }
    catch (ios_base::failure ex)
    {
      throw IOException(ex.what());
    }
}

void Properties::print()
{
    for (map<string,string>::const_iterator it=data.begin(); it!=data.end(); it++)
      cout << it->first << "=" << it->second << endl;
}

}
