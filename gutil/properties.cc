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

#include "properties.h"
#include "misc.h"

#include <gmath/dmatrix.h>

#include <fstream>
#include <memory>
#include <iostream>

namespace gutil
{

bool Properties::operator == (const Properties &p) const
{
  bool ret=true;

  if (data.size() == p.data.size())
  {
    for (std::map<std::string, std::string>::const_iterator it=data.begin(); it!=data.end(); ++it)
    {
      std::map<std::string, std::string>::const_iterator pit=p.data.find(it->first);

      if (pit == data.end() || it->second != pit->second)
      {
        ret=false;
        break;
      }
    }
  }
  else
  {
    ret=false;
  }

  return ret;
}

void Properties::getString(const char *key, std::string &value,
                           const char *defvalue) const
{
  std::map<std::string, std::string>::const_iterator it=data.find(key);

  if (it != data.end())
  {
    value=it->second;
  }
  else if (defvalue != 0)
  {
    value=defvalue;
  }
  else
  {
    throw IOException("Key not found: "+std::string(key));
  }
}

void Properties::getStringVector(const char *key, std::vector<std::string> &value,
                                 const char *defvalue, const char sep) const
{
  std::string s;
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

void Properties::putString(const char *key, const std::string &value)
{
  std::map<std::string, std::string>::iterator it=data.find(key);

  if (it != data.end())
  {
    data.erase(key);
  }

  data.insert(std::pair<std::string, std::string>(key, value));
}

void Properties::load(const char *name)
{
  std::ifstream in;
  in.open(name);
  load(in, name);
}

void Properties::load(std::istream &in)
{
  load(in, "stream");
}

void Properties::save(const char *name, const char *comment) const
{
  std::ofstream out;

  try
  {
    out.exceptions(std::ios_base::failbit | std::ios_base::badbit | std::ios_base::eofbit);
    out.open(name);

    save(out, comment);

    out.close();
  }
  catch (const std::ios_base::failure &ex)
  {
    throw IOException(ex.what());
  }
}

void Properties::save(std::ostream& out, const char *comment) const
{
  if (comment != 0)
  {
    out << "# " << comment << std::endl;
  }

  for (std::map<std::string, std::string>::const_iterator it=data.begin(); it!=data.end(); ++it)
  {
    out << it->first << "=" << it->second << std::endl;
  }
}

void Properties::print()
{
  for (std::map<std::string, std::string>::const_iterator it=data.begin(); it!=data.end(); ++it)
  {
    std::cout << it->first << "=" << it->second << std::endl;
  }
}

class PropertyNode
{
  public:

    PropertyNode(const std::string &_name)
    {
      if (_name.size() > 0 && std::isdigit(_name[0]))
      {
        // name must not begin with a digit!
        name="x";
      }

      name.append(_name);
    }

    void addChild(const std::string &_name, const std::string &_value)
    {
      size_t i=_name.find_first_of('.');

      if (i != std::string::npos)
      {
        getChild(_name.substr(0, i))->addChild(_name.substr(i+1), _value);
      }
      else
      {
        getChild(_name)->setValue(_value);
      }
    }

    const std::string &getName() const
    {
      return name;
    }

    void setValue(const std::string &_value)
    {
      value=_value;
    }

    void saveOctave(std::ostream &out) const
    {
      if (list.size() > 0)
      {
        if (name.size() > 0)
        {
          // add structure element
          out << "# name: " << name << std::endl;
          out << "# type: scalar struct" << std::endl;
          out << "# ndims: 2" << std::endl;
          out << "  1 1" << std::endl;
          out << "# length: " << list.size() << std::endl;
        }

        // store all children
        for (size_t i=0; i<list.size(); i++)
        {
          list[i]->saveOctave(out);
        }

        out << std::endl;
        out << std::endl;
      }
      else if (name.size() > 0 && value.size() > 0)
      {
        char *vp=0;
        std::strtod(value.c_str(), &vp);

        if (*vp == '\0')
        {
          // store scalar
          out << "# name: " << name << std::endl;
          out << "# type: scalar" << std::endl;
          out << value << std::endl;
          out << std::endl;
          out << std::endl;
        }
        else
        {
          try
          {
            gmath::Matrixd M;
            std::istringstream in(value);
            in >> M;

            if (M.rows() > 0 && M.cols() > 0)
            {
              if (M.rows() == 1)
              {
                // store vector always as column
                gmath::Matrixd tmp=M;
                M=gmath::transpose(tmp);
              }

              // store matrix
              out << "# name: " << name << std::endl;
              out << "# type: matrix" << std::endl;
              out << "# rows: " << M.rows() << std::endl;
              out << "# columns: " << M.cols() << std::endl;
              out.precision(16);

              for (int k=0; k<M.rows(); k++)
              {
                for (int i=0; i<M.cols(); i++)
                {
                  out << ' ' << M(k, i);
                }

                out << std::endl;
              }

              out << std::endl;
              out << std::endl;
            }
          }
          catch (...)
          { }
        }
      }
    }

  private:

    std::shared_ptr<PropertyNode> &getChild(const std::string &_name)
    {
      for (size_t i=0; i<list.size(); i++)
      {
        if (_name == list[i]->getName())
        {
          return list[i];
        }
      }

      list.push_back(std::make_shared<PropertyNode>(_name));

      return list.back();
    }

    std::string name;
    std::string value;
    std::vector<std::shared_ptr<PropertyNode> > list;
};

void Properties::saveOctave(std::ostream& out, const char *comment) const
{
  if (comment == 0)
  {
    comment="Converted properties";
  }

  std::shared_ptr<PropertyNode> root=std::make_shared<PropertyNode>(std::string());

  for (std::map<std::string, std::string>::const_iterator it=data.begin(); it!=data.end(); ++it)
  {
    root->addChild(it->first, it->second);
  }

  out << "# " << comment << std::endl;
  root->saveOctave(out);
}

void Properties::load(std::istream &in, const char *name)
{
  std::string line;

  try
  {
    in.exceptions(std::ios_base::badbit);

    if (!in.good())
    {
      throw IOException(name);
    }

    while (in.good())
    {
      getline(in, line);

      trim(line);

      size_t pos=line.find('#');

      if (pos != line.npos)
      {
        line=line.substr(0, pos);
      }

      if (line.size() > 0)
      {
        pos=line.find('=');

        if (pos != line.npos)
        {
          std::string key=line.substr(0, pos);
          std::string value=line.substr(pos+1);

          trim(key);
          trim(value);

          data.insert(std::pair<std::string,std::string>(key, value));
        }
        else
        {
          throw IOException("Format <key>=<value> expected: "+line);
        }
      }
    }
  }
  catch (const std::ios_base::failure &ex)
  {
    throw IOException(ex.what());
  }
}

}
