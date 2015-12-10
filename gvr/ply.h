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

#ifndef GVR_PLY_H
#define GVR_PLY_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <gutil/exception.h>

namespace gvr
{

enum ply_encoding { ply_ascii, ply_binary, ply_big_endian, ply_little_endian };

enum ply_type { ply_none, ply_int8, ply_uint8, ply_int16, ply_uint16,
  ply_int32, ply_uint32, ply_float32, ply_float64 };

/**
 * Class for offering read and write access to ply values.
 */

class PLYValue
{
  private:
  
    PLYValue(const PLYValue &p) {}
    PLYValue &operator=(const PLYValue &p) { return *this; }
  
  public:
    
    PLYValue() {}
    virtual ~PLYValue() {}
    
      // reading a value from the stream
    
    virtual void read(std::streambuf *in)=0;
    
      // returning the value
    
    virtual int getListSize() const { return 0; }
    virtual int getInt(int i=0) const=0;
    virtual unsigned int getUnsignedInt(int i=0) const=0;
    virtual float getFloat(int i=0) const=0;
    virtual double getDouble(int i=0) const=0;
    
      // writing the value to a stream
    
    virtual void writeListSize(std::streambuf *out, int v);
    virtual void write(std::streambuf *out, int v)=0;
    virtual void write(std::streambuf *out, unsigned int v)=0;
    virtual void write(std::streambuf *out, float v)=0;
    virtual void write(std::streambuf *out, double v)=0;
};

/**
 * Receiver for loading data.
 */

class PLYReceiver
{
  public:
  
    virtual void setValue(int instance, const PLYValue &value)=0;
};

class PLYElement;

/**
 * Class for loading a ply file.
 */

class PLYReader
{
  private:
    
    std::string filename;
    std::ifstream in;
    std::vector<PLYElement *> list;
    
    PLYElement *findElement(const std::string &name) const;
    
  public:
    
    ~PLYReader();
    
      // opens the ply file and reads the header, false is returned if the file
      // is not a ply file
    
    bool open(const char *name);
    
      // print header to stdout
    
    void printHeader();
    
      // get name of ply file
    
    const char *getName() { return filename.c_str(); }
    
      // get all comments
    
    void getComments(std::vector<std::string> &clist) const;
    
      // returns the instances of the requested element or -1 if the element
      // does not exist
    
    long instancesOfElement(const std::string &elem_name) const;
    
      // returns the type of the requested property or ply_none if the property
      // does not exist
    
    ply_type getTypeOfProperty(const std::string &elem_name, const std::string &prop_name) const;
    
      // sets a receiver object to a property and returns true if the property
      // exists
    
    bool setReceiver(const std::string &elem_name, const std::string &prop_name, PLYReceiver *receiver);
    
      // reads data of all elements, calls the associated receivers and closes
      // the file
    
    void readData();
};

/**
 * Class for writing a ply file. All four steps have to be done after each
 * other.
 */

class PLYWriter
{
  private:
  
    ply_encoding              enc;
    std::vector<PLYElement *> list;
    bool                      header_complete;
    PLYElement                *element;
    size_t                    pelem;
    long                      pinst;
    int                       pprop;
    int                       plist;
    std::ofstream             out;
    
    void writeHeader();
    PLYValue &nextValue();
    
  public:
    
    ~PLYWriter();
    
      // step 1: open a new ply file for writing
    
    void open(const char *name, ply_encoding encoding=ply_binary);
    
      // step 2: specification of comments, elements and properties
    
    void addComment(const std::string &s);
    
      // an element contains a list of properties that define together one
      // instance of data, there can be several elements
    
    void addElement(const std::string &name, long instances);
    
      // each element contains one or more properties
    
    void addProperty(const std::string &name, ply_type tvalue);
    void addProperty(const std::string &name, ply_type tsize, ply_type tvalue);
    
      // step 3: writes the data for each instance of element and each property
      // (must be called in exactly the same order as specified in the header,
      // an exception is thrown if more data is written than specified in the
      // header)
    
    void writeListSize(int v);
    void write(int v);
    void write(unsigned int v);
    void write(float v);
    void write(double v);
    
      // step 4: closing the file (this will throw an exception if the data
      // that has been specified in the header is not written completely)
    
    void close();
};

}

#endif
