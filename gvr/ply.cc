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

#include "ply.h"
#include <gutil/misc.h>
#include <gutil/exception.h>

#include <sstream>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <assert.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::ostringstream;
using std::istringstream;
using std::streambuf;
using std::ios_base;

using gutil::isMSBFirst;
using gutil::split;
using gutil::IOException;
using gutil::trim;

namespace gvr
{

void PLYValue::writeListSize(streambuf *out, int v)
{
    assert(false);
}

namespace
{

void readASCIIElement(streambuf *in, char *out, int n)
{
      // eat white spaces
    
    int c=in->sgetc();
    while (c != EOF && isspace(c))
      c=in->snextc();
    
      // read until next white space or end of file
    
    n--;
    int i=0;
    while (c != EOF && !isspace(c))
    {
      if (i < n)
        out[i++]=static_cast<char>(c);
      
      c=in->snextc();
    }
    
    out[i]='\0';
}

/*
  Implementations for reading and writing values of all types and encodings.
*/

class PLYValueASCIIInt : public PLYValue
{
  private:
  
    int value;
  
  public:
  
    void read(streambuf *in)
    {
      char s[40];
      readASCIIElement(in, s, 40);
      value=atoi(s);
    }

    int getInt(int i=0) const { return value; }
    unsigned int getUnsignedInt(int i=0) const { return static_cast<unsigned int>(value); }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, int v)
    {
      char s[40];
      sprintf(s, "%d ", v);
      out->sputn(s, strlen(s));
    }
    
    void write(streambuf *out, unsigned int v) { write(out, static_cast<int>(v)); }
    void write(streambuf *out, float v) { write(out, static_cast<int>(v)); }
    void write(streambuf *out, double v) { write(out, static_cast<int>(v)); }
};

class PLYValueASCIIUInt : public PLYValue
{
  private:
  
    unsigned int value;
  
  public:
  
    void read(streambuf *in)
    {
      char s[40];
      readASCIIElement(in, s, 40);
      value=static_cast<unsigned int>(atol(s));
    }
    
    int getInt(int i=0) const { return static_cast<int>(value); }
    unsigned int getUnsignedInt(int i=0) const { return value; }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, int v) { write(out, static_cast<unsigned int>(v)); }
    
    void write(streambuf *out, unsigned int v)
    {
      char s[40];
      sprintf(s, "%u ", v);
      out->sputn(s, strlen(s));
    }
    
    void write(streambuf *out, float v) { write(out, static_cast<unsigned int>(v)); }
    void write(streambuf *out, double v) { write(out, static_cast<unsigned int>(v)); }
};

class PLYValueASCIIDouble : public PLYValue
{
  private:
  
    double value;
  
  public:
  
    void read(streambuf *in)
    {
      char s[40];
      readASCIIElement(in, s, 40);
      value=atof(s);
    }
    
    int getInt(int i=0) const { return static_cast<int>(value); }
    unsigned int getUnsignedInt(int i=0) const { return static_cast<unsigned int>(value); }
    float getFloat(int i=0) const { return static_cast<float>(value); }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, int v) { write(out, static_cast<double>(v)); }
    void write(streambuf *out, unsigned int v) { write(out, static_cast<double>(v)); }
    
    void write(streambuf *out, float v)
    {
      char s[40];
      sprintf(s, "%.8g ", v);
      out->sputn(s, strlen(s));
    }
    
    void write(streambuf *out, double v)
    {
      char s[40];
      sprintf(s, "%.16g ", v);
      out->sputn(s, strlen(s));
    }
};

class PLYValueInt8 : public PLYValue
{
  private:
  
    char value;
  
  public:
  
    void read(streambuf *in) { value=static_cast<char>(in->sbumpc()); }
    
    int getInt(int i=0) const { return value; }
    unsigned int getUnsignedInt(int i=0) const { return static_cast<unsigned int>(value); }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, int v) { out->sputc(static_cast<char>(v)); }
    void write(streambuf *out, unsigned int v) { out->sputc(static_cast<char>(v)); }
    void write(streambuf *out, float v) { value=static_cast<char>(v); out->sputc(value); }
    void write(streambuf *out, double v) { value=static_cast<char>(v); out->sputc(value); }
};

class PLYValueUInt8 : public PLYValue
{
  private:
  
    unsigned char value;
  
  public:
  
    void read(streambuf *in) { value=static_cast<unsigned char>(in->sbumpc()); }
    
    int getInt(int i=0) const { return value; }
    unsigned int getUnsignedInt(int i=0) const { return value; }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, int v) { out->sputc(static_cast<unsigned char>(v)); }
    void write(streambuf *out, unsigned int v) { out->sputc(static_cast<unsigned char>(v)); }
    void write(streambuf *out, float v) { value=static_cast<unsigned char>(v); out->sputc(static_cast<char>(value)); }
    void write(streambuf *out, double v) { value=static_cast<unsigned char>(v); out->sputc(static_cast<char>(value)); }
};

class PLYValueBigInt16 : public PLYValue
{
  private:
  
    short value;
  
  public:
  
    void read(streambuf *in)
    {
      value=static_cast<short>(in->sbumpc()<<8);
      value|=static_cast<short>(in->sbumpc()&0xff);
    }
    
    int getInt(int i=0) const { return value; }
    unsigned int getUnsignedInt(int i=0) const { return static_cast<unsigned int>(value); }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, int v)
    {
      out->sputc(static_cast<char>((v>>8)&0xff));
      out->sputc(static_cast<char>(v&0xff));
    }
    
    void write(streambuf *out, unsigned int v) { write(out, static_cast<int>(v)); }
    void write(streambuf *out, float v) { write(out, static_cast<int>(v)); }
    void write(streambuf *out, double v) { write(out, static_cast<int>(v)); }
};

class PLYValueBigUInt16 : public PLYValue
{
  private:
  
    unsigned short value;
  
  public:
  
    void read(streambuf *in)
    {
      value=static_cast<unsigned short>((in->sbumpc()&0xff)<<8);
      value|=static_cast<unsigned short>(in->sbumpc()&0xff);
    }
    
    int getInt(int i=0) const { return static_cast<int>(value); }
    unsigned int getUnsignedInt(int i=0) const { return value; }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, unsigned int v)
    {
      out->sputc(static_cast<char>((v>>8)&0xff));
      out->sputc(static_cast<char>(v&0xff));
    }
    
    void write(streambuf *out, int v) { write(out, static_cast<unsigned int>(v)); }
    void write(streambuf *out, float v) { write(out, static_cast<unsigned int>(v)); }
    void write(streambuf *out, double v) { write(out, static_cast<unsigned int>(v)); }
};

class PLYValueBigInt32 : public PLYValue
{
  private:
  
    int value;
  
  public:
  
    void read(streambuf *in)
    {
      value=in->sbumpc()<<24;
      value|=(in->sbumpc()&0xff)<<16;
      value|=(in->sbumpc()&0xff)<<8;
      value|=in->sbumpc()&0xff;
    }
    
    int getInt(int i=0) const { return value; }
    unsigned int getUnsignedInt(int i=0) const { return static_cast<unsigned int>(value); }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, int v)
    {
      out->sputc(static_cast<char>((v>>24)&0xff));
      out->sputc(static_cast<char>((v>>16)&0xff));
      out->sputc(static_cast<char>((v>>8)&0xff));
      out->sputc(static_cast<char>(v&0xff));
    }
    
    void write(streambuf *out, unsigned int v) { write(out, static_cast<int>(v)); }
    void write(streambuf *out, float v) { write(out, static_cast<int>(v)); }
    void write(streambuf *out, double v) { write(out, static_cast<int>(v)); }
};

class PLYValueBigUInt32 : public PLYValue
{
  private:
  
    unsigned int value;
  
  public:
  
    void read(streambuf *in)
    {
      value=(in->sbumpc()&0xff)<<24;
      value|=(in->sbumpc()&0xff)<<16;
      value|=(in->sbumpc()&0xff)<<8;
      value|=in->sbumpc()&0xff;
    }
    
    int getInt(int i=0) const { return static_cast<unsigned int>(value); }
    unsigned int getUnsignedInt(int i=0) const { return value; }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, unsigned int v)
    {
      out->sputc(static_cast<char>((v>>24)&0xff));
      out->sputc(static_cast<char>((v>>16)&0xff));
      out->sputc(static_cast<char>((v>>8)&0xff));
      out->sputc(static_cast<char>(v&0xff));
    }
    
    void write(streambuf *out, int v) { write(out, static_cast<unsigned int>(v)); }
    void write(streambuf *out, float v) { write(out, static_cast<unsigned int>(v)); }
    void write(streambuf *out, double v) { write(out, static_cast<unsigned int>(v)); }
};

class PLYValueLittleInt16 : public PLYValue
{
  private:
  
    short value;
  
  public:
  
    void read(streambuf *in)
    {
      value=static_cast<short>(in->sbumpc()&0xff);
      value|=static_cast<short>(in->sbumpc()<<8);
    }
    
    int getInt(int i=0) const { return value; }
    unsigned int getUnsignedInt(int i=0) const { return static_cast<unsigned int>(value); }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, int v)
    {
      out->sputc(static_cast<char>(v&0xff));
      out->sputc(static_cast<char>((v>>8)&0xff));
    }
    
    void write(streambuf *out, unsigned int v) { write(out, static_cast<int>(v)); }
    void write(streambuf *out, float v) { write(out, static_cast<int>(v)); }
    void write(streambuf *out, double v) { write(out, static_cast<int>(v)); }
};

class PLYValueLittleUInt16 : public PLYValue
{
  private:
  
    unsigned short value;
  
  public:
  
    void read(streambuf *in)
    {
      value=static_cast<unsigned short>(in->sbumpc()&0xff);
      value|=static_cast<unsigned short>((in->sbumpc()&0xff)<<8);
    }
    
    int getInt(int i=0) const { return static_cast<int>(value); }
    unsigned int getUnsignedInt(int i=0) const { return value; }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, unsigned int v)
    {
      out->sputc(static_cast<char>(v&0xff));
      out->sputc(static_cast<char>((v>>8)&0xff));
    }
    
    void write(streambuf *out, int v) { write(out, static_cast<unsigned int>(v)); }
    void write(streambuf *out, float v) { write(out, static_cast<unsigned int>(v)); }
    void write(streambuf *out, double v) { write(out, static_cast<unsigned int>(v)); }
};

class PLYValueLittleInt32 : public PLYValue
{
  private:
  
    int value;
  
  public:
  
    void read(streambuf *in)
    {
      value=in->sbumpc()&0xff;
      value|=(in->sbumpc()&0xff)<<8;
      value|=(in->sbumpc()&0xff)<<16;
      value|=(in->sbumpc()&0xff)<<24;
    }
    
    int getInt(int i=0) const { return value; }
    unsigned int getUnsignedInt(int i=0) const { return static_cast<unsigned int>(value); }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, int v)
    {
      out->sputc(static_cast<char>(v&0xff));
      out->sputc(static_cast<char>((v>>8)&0xff));
      out->sputc(static_cast<char>((v>>16)&0xff));
      out->sputc(static_cast<char>((v>>24)&0xff));
    }
    
    void write(streambuf *out, unsigned int v) { write(out, static_cast<int>(v)); }
    void write(streambuf *out, float v) { write(out, static_cast<int>(v)); }
    void write(streambuf *out, double v) { write(out, static_cast<int>(v)); }
};

class PLYValueLittleUInt32 : public PLYValue
{
  private:
  
    unsigned int value;
  
  public:
  
    void read(streambuf *in)
    {
      value=in->sbumpc()&0xff;
      value|=(in->sbumpc()&0xff)<<8;
      value|=(in->sbumpc()&0xff)<<16;
      value|=(in->sbumpc()&0xff)<<24;
    }
    
    int getInt(int i=0) const { return static_cast<unsigned int>(value); }
    unsigned int getUnsignedInt(int i=0) const { return value; }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, unsigned int v)
    {
      out->sputc(static_cast<char>(v&0xff));
      out->sputc(static_cast<char>((v>>8)&0xff));
      out->sputc(static_cast<char>((v>>16)&0xff));
      out->sputc(static_cast<char>((v>>24)&0xff));
    }
    
    void write(streambuf *out, int v) { write(out, static_cast<unsigned int>(v)); }
    void write(streambuf *out, float v) { write(out, static_cast<unsigned int>(v)); }
    void write(streambuf *out, double v) { write(out, static_cast<unsigned int>(v)); }
};

class PLYValueFloat32 : public PLYValue
{
  private:
  
    float value;
  
  public:
  
    void read(streambuf *in)
    {
      char *p=reinterpret_cast<char *>(&value);
      p[0]=static_cast<char>(in->sbumpc());
      p[1]=static_cast<char>(in->sbumpc());
      p[2]=static_cast<char>(in->sbumpc());
      p[3]=static_cast<char>(in->sbumpc());
    }
    
    int getInt(int i=0) const { return static_cast<int>(value); }
    unsigned int getUnsignedInt(int i=0) const { return static_cast<unsigned int>(value); }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, float v)
    {
      char *p=reinterpret_cast<char *>(&v);
      out->sputc(p[0]);
      out->sputc(p[1]);
      out->sputc(p[2]);
      out->sputc(p[3]);
    }
    
    void write(streambuf *out, unsigned int v) { write(out, static_cast<float>(v)); }
    void write(streambuf *out, int v) { write(out, static_cast<float>(v)); }
    void write(streambuf *out, double v) { write(out, static_cast<float>(v)); }
};

class PLYValueFloat64 : public PLYValue
{
  private:
  
    double value;
  
  public:
  
    void read(streambuf *in)
    {
      char *p=reinterpret_cast<char *>(&value);
      p[0]=static_cast<char>(in->sbumpc());
      p[1]=static_cast<char>(in->sbumpc());
      p[2]=static_cast<char>(in->sbumpc());
      p[3]=static_cast<char>(in->sbumpc());
      p[4]=static_cast<char>(in->sbumpc());
      p[5]=static_cast<char>(in->sbumpc());
      p[6]=static_cast<char>(in->sbumpc());
      p[7]=static_cast<char>(in->sbumpc());
    }
    
    int getInt(int i=0) const { return static_cast<int>(value); }
    unsigned int getUnsignedInt(int i=0) const { return static_cast<unsigned int>(value); }
    float getFloat(int i=0) const { return static_cast<float>(value); }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, double v)
    {
      char *p=reinterpret_cast<char *>(&v);
      out->sputc(p[0]);
      out->sputc(p[1]);
      out->sputc(p[2]);
      out->sputc(p[3]);
      out->sputc(p[4]);
      out->sputc(p[5]);
      out->sputc(p[6]);
      out->sputc(p[7]);
    }
    
    void write(streambuf *out, unsigned int v) { write(out, static_cast<double>(v)); }
    void write(streambuf *out, int v) { write(out, static_cast<double>(v)); }
    void write(streambuf *out, float v) { write(out, static_cast<double>(v)); }
};

class PLYValueSwapFloat32 : public PLYValue
{
  private:
  
    float value;
  
  public:
  
    void read(streambuf *in)
    {
      char *p=reinterpret_cast<char *>(&value);
      p[3]=static_cast<char>(in->sbumpc());
      p[2]=static_cast<char>(in->sbumpc());
      p[1]=static_cast<char>(in->sbumpc());
      p[0]=static_cast<char>(in->sbumpc());
    }
    
    int getInt(int i=0) const { return static_cast<int>(value); }
    unsigned int getUnsignedInt(int i=0) const { return static_cast<unsigned int>(value); }
    float getFloat(int i=0) const { return value; }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, float v)
    {
      char *p=reinterpret_cast<char *>(&v);
      out->sputc(p[3]);
      out->sputc(p[2]);
      out->sputc(p[1]);
      out->sputc(p[0]);
    }
    
    void write(streambuf *out, unsigned int v) { write(out, static_cast<float>(v)); }
    void write(streambuf *out, int v) { write(out, static_cast<float>(v)); }
    void write(streambuf *out, double v) { write(out, static_cast<float>(v)); }
};


class PLYValueSwapFloat64 : public PLYValue
{
  private:
  
    double value;
  
  public:
  
    void read(streambuf *in)
    {
      char *p=reinterpret_cast<char *>(&value);
      p[7]=static_cast<char>(in->sbumpc());
      p[6]=static_cast<char>(in->sbumpc());
      p[5]=static_cast<char>(in->sbumpc());
      p[4]=static_cast<char>(in->sbumpc());
      p[3]=static_cast<char>(in->sbumpc());
      p[2]=static_cast<char>(in->sbumpc());
      p[1]=static_cast<char>(in->sbumpc());
      p[0]=static_cast<char>(in->sbumpc());
    }
    
    int getInt(int i=0) const { return static_cast<int>(value); }
    unsigned int getUnsignedInt(int i=0) const { return static_cast<unsigned int>(value); }
    float getFloat(int i=0) const { return static_cast<float>(value); }
    double getDouble(int i=0) const { return value; }
    
    void write(streambuf *out, double v)
    {
      char *p=reinterpret_cast<char *>(&v);
      out->sputc(p[7]);
      out->sputc(p[6]);
      out->sputc(p[5]);
      out->sputc(p[4]);
      out->sputc(p[3]);
      out->sputc(p[2]);
      out->sputc(p[1]);
      out->sputc(p[0]);
    }
    
    void write(streambuf *out, unsigned int v) { write(out, static_cast<double>(v)); }
    void write(streambuf *out, int v) { write(out, static_cast<double>(v)); }
    void write(streambuf *out, float v) { write(out, static_cast<double>(v)); }
};

/*
  Function for creating a value object according to a given type and encoding.
*/

PLYValue *createValue(ply_type type, ply_encoding encoding)
{
    if (encoding == ply_ascii)
    {
      switch (type)
      {
        case ply_int8:
        case ply_int16:
        case ply_int32:
            return new PLYValueASCIIInt();
        
        case ply_uint8:
        case ply_uint16:
        case ply_uint32:
            return new PLYValueASCIIUInt();
        
        case ply_float32:
        case ply_float64:
            return new PLYValueASCIIDouble();
        
        default:
            break;
      }
    }
    else if (encoding == ply_big_endian)
    {
      switch (type)
      {
        case ply_int8:
            return new PLYValueInt8();
        
        case ply_uint8:
            return new PLYValueUInt8();
        
        case ply_int16:
            return new PLYValueBigInt16();
        
        case ply_uint16:
            return new PLYValueBigUInt16();
        
        case ply_int32:
            return new PLYValueBigInt32();
        
        case ply_uint32:
            return new PLYValueBigUInt32();
        
        case ply_float32:
            if (isMSBFirst())
              return new PLYValueFloat32();
            else
              return new PLYValueSwapFloat32();
        
        case ply_float64:
            if (isMSBFirst())
              return new PLYValueFloat64();
            else
              return new PLYValueSwapFloat64();
        
        default:
            break;
      }
    }
    else if (encoding == ply_little_endian)
    {
      switch (type)
      {
        case ply_int8:
            return new PLYValueInt8();
        
        case ply_uint8:
            return new PLYValueUInt8();
        
        case ply_int16:
            return new PLYValueLittleInt16();
        
        case ply_uint16:
            return new PLYValueLittleUInt16();
        
        case ply_int32:
            return new PLYValueLittleInt32();
        
        case ply_uint32:
            return new PLYValueLittleUInt32();
        
        case ply_float32:
            if (isMSBFirst())
              return new PLYValueSwapFloat32();
            else
              return new PLYValueFloat32();
        
        case ply_float64:
            if (isMSBFirst())
              return new PLYValueSwapFloat64();
            else
              return new PLYValueFloat64();
        
        default:
            break;
      }
    }
    
    assert(false);
    
    return 0;
}

/*
  The ply list property is implemented as an object that consists of a size and
  a list of values.
*/

class PLYValueList : public PLYValue
{
  private:
  
    ply_encoding       enc;
    
    PLYValue           *size;
    
    ply_type           tvalue;
    size_t             used;
    vector<PLYValue *> list;
  
  public:
    
    PLYValueList(ply_type t_size, ply_type t_value, ply_encoding encoding)
    {
      enc=encoding;
      size=createValue(t_size, encoding);
      tvalue=t_value;
      used=0;
      list.resize(1);
      list[0]=createValue(t_value, encoding);
    }
    
    ~PLYValueList()
    {
      delete size;
      
      for (size_t i=0; i<list.size(); i++)
        delete list[i];
    }
    
    void read(streambuf *in)
    {
      size->read(in);
      
      if (size->getUnsignedInt() > list.size())
      {
        used=list.size();
        list.resize(size->getUnsignedInt());
        
        for (size_t i=used; i<list.size(); i++)
          list[i]=createValue(tvalue, enc);
      }
      
      used=size->getUnsignedInt();
      for (size_t i=0; i<used; i++)
        list[i]->read(in);
    }
    
    int getListSize() const { return used; }
    int getInt(int i=0) const { return list[i]->getInt(); }
    unsigned int getUnsignedInt(int i=0) const { return list[i]->getUnsignedInt(); }
    float getFloat(int i=0) const { return list[i]->getFloat(); }
    double getDouble(int i=0) const { return list[i]->getDouble(); }
    
    void writeListSize(streambuf *out, int v) { size->write(out, v); }
    void write(streambuf *out, int v) { list[0]->write(out, v); }
    void write(streambuf *out, unsigned int v) { list[0]->write(out, v); }
    void write(streambuf *out, float v) { list[0]->write(out, v); }
    void write(streambuf *out, double v) { list[0]->write(out, v); }
};

/*
  PLY name property, which manages one value or a list of values of a certain
  type and handles an optional receiver object.
*/

class PLYProperty
{
  private:
  
    ply_encoding enc;
    string       name;
    ply_type     tsize;
    ply_type     tvalue;
    PLYValue     *value;
    PLYReceiver  *receiver;
    
    static string type2Name(ply_type type);
    static ply_type name2Type(const string &name);
    
  public:
  
      // construction of a scalar or list property
    
    PLYProperty();
    PLYProperty(const string &propname, ply_type type, ply_encoding encoding);
    PLYProperty(const string &propname, ply_type t_size, ply_type t_value, ply_encoding encoding);
    PLYProperty(const PLYProperty &p);
    ~PLYProperty();
    
    PLYProperty &operator=(const PLYProperty &p);
    
    static PLYProperty fromString(const string &s, ply_encoding encoding);
    
      // returns the specification as in the ply header
    
    string toString() const;
    
      // returns name and the type of property (tsize is ply_none, if the
      // property is not a list)
    
    const string &getName() const { return name; }
    ply_type getSizeType() const { return tsize; }
    ply_type getValueType() const { return tvalue; }
    PLYValue &getValue() { return *value; }
    
      // sets a receiver object
    
    void setReceiver(PLYReceiver *r) { receiver=r; }
    
      // reads one value and calls the receiver object if specified
    
    void readData(streambuf *in, int instance);
};

string PLYProperty::type2Name(ply_type type)
{
    switch (type)
    {
      case ply_int8:
          return "int8";
      
      case ply_uint8:
          return "uint8";
      
      case ply_int16:
          return "int16";
      
      case ply_uint16:
          return "uint16";
      
      case ply_int32:
          return "int32";
      
      case ply_uint32:
          return "uint32";
      
      case ply_float32:
          return "float32";
      
      case ply_float64:
          return "float64";
      
      default:
          break;
    }
    
    assert(false);
    
    return "none";
}

ply_type PLYProperty::name2Type(const string &name)
{
    if (name == "int8" || name == "char")
      return ply_int8;
    else if (name == "uint8" || name == "uchar")
      return ply_uint8;
    else if (name == "int16" || name == "short")
      return ply_int16;
    else if (name == "uint16" || name == "ushort")
      return ply_uint16;
    else if (name == "int32" || name == "int")
      return ply_int32;
    else if (name == "uint32" || name == "uint")
      return ply_uint32;
    else if (name == "float32" || name == "float")
      return ply_float32;
    else if (name == "float64" || name == "double")
      return ply_float64;
    
    assert(false);
    
    return ply_none;
}

PLYProperty::PLYProperty()
{
    enc=ply_ascii;
    name="";
    tsize=ply_none;
    tvalue=ply_none;
    value=0;
    receiver=0;
}

PLYProperty::PLYProperty(const string &propname, ply_type type, ply_encoding encoding)
{
    enc=encoding;
    name=propname;
    tsize=ply_none;
    tvalue=type;
    value=createValue(type, encoding);
    receiver=0;
}

PLYProperty::PLYProperty(const string &propname, ply_type t_size, ply_type t_value, ply_encoding encoding)
{
    enc=encoding;
    name=propname;
    tsize=t_size;
    tvalue=t_value;
    value=new PLYValueList(tsize, tvalue, encoding);
    receiver=0;
}

PLYProperty::PLYProperty(const PLYProperty &p)
{
    enc=p.enc;
    name=p.name;
    tsize=p.tsize;
    tvalue=p.tvalue;
    
    if (tsize == ply_none)
      value=createValue(tvalue, enc);
    else
      value=new PLYValueList(tsize, tvalue, enc);
    
    receiver=0;
}

PLYProperty::~PLYProperty()
{
    delete value;
}

PLYProperty &PLYProperty::operator=(const PLYProperty &p)
{
    enc=p.enc;
    name=p.name;
    tsize=p.tsize;
    tvalue=p.tvalue;
    
    delete value;
    
    if (tsize == ply_none)
      value=createValue(tvalue, enc);
    else
      value=new PLYValueList(tsize, tvalue, enc);
    
    receiver=p.receiver;
    
    return *this;
}

PLYProperty PLYProperty::fromString(const string &s, ply_encoding encoding)
{
    vector<string> list;
    
    split(list, s);
    
    if (list[0].compare("property") != 0)
      throw IOException("Invalid PLY property definition: "+s);
    
    if (list.size() == 3)
    {
      return PLYProperty(list[2], name2Type(list[1]), encoding);
    }
    else if (list.size() == 5 && list[1].compare("list") == 0)
    {
      return PLYProperty(list[4], name2Type(list[2]), name2Type(list[3]), encoding);
    }
    else
      throw IOException("Invalid PLY property definition: "+s);
}

string PLYProperty::toString() const
{
    if (tsize == ply_none)
      return "property "+type2Name(tvalue)+" "+name;
    else
      return "property list "+type2Name(tsize)+" "+type2Name(tvalue)+" "+name;
}

void PLYProperty::readData(streambuf *in, int instance)
{
    value->read(in);
    
    if (receiver != 0)
      receiver->setValue(instance, *value);
}

}

/*
 * PLY comment (i.e. if size < 0) or element that manages a list of properties.
 */

class PLYElement
{
  private:
  
    string              name;
    long                size;
    vector<PLYProperty> list;
    
  public:
    
      // construction of an element
    
    PLYElement(const string &elem_name, long elem_size);
    
    static PLYElement *fromString(const string &s);
    
      // returns the specification as in the ply header
    
    string toString() const;
    
      // returns the element name and number of instances (if the number of
      // instances is < 0, then this is a comment)
    
    const string &getName() const { return name; }
    long getInstances() const { return size; }
    
      // adds a property to the element
    
    void addProperty(const PLYProperty &p) { list.push_back(p); }
    
      // returns the associated properties
    
    int getPropertyCount() { return list.size(); }
    PLYProperty &getProperty(int i) { return list[i]; }
    PLYProperty *findProperty(const string s);
    
      // reading all data (i.e. all instances) of this element and calls the
      // receiver objects if specified
    
    void readData(streambuf *in);
};

PLYElement::PLYElement(const string &elem_name, long elem_size)
{
    name=elem_name;
    size=elem_size;
}

PLYElement *PLYElement::fromString(const string &s)
{
    vector<string> list;
    
    split(list, s);
    
    if (list[0].compare("element") == 0)
    {
      long n;
      istringstream in(list[2]);
      in >> n;
      
      return new PLYElement(list[1], n);
    }
    else if (list[0].compare("comment") == 0)
    {
      return new PLYElement(s.substr(8), -1);
    }
    else
      throw IOException("Invalid PLY element definition: "+s);
}

string PLYElement::toString() const
{
    ostringstream s;
    
    if (size >= 0)
      s << "element " << name << " " << size;
    else
      s << "comment " << name;
    
    return s.str();
}

PLYProperty *PLYElement::findProperty(const string s)
{
    PLYProperty *ret=0;
    
    for (size_t i=0; i<list.size() && ret == 0; i++)
    {
      if (list[i].getName().compare(s) == 0)
        ret=&list[i];
    }
    
    return ret;
}

void PLYElement::readData(streambuf *in)
{
    for (long i=0; i<size; i++)
    {
      for (size_t k=0; k<list.size(); k++)
        list[k].readData(in, i);
    }
}

PLYElement *PLYReader::findElement(const string &name) const
{
    PLYElement *ret=0;
    
    for (size_t i=0; i<list.size() && ret == 0; i++)
    {
      if (list[i]->getName().compare(name) == 0)
        ret=list[i];
    }
    
    return ret;
}

PLYReader::~PLYReader()
{
    for (size_t i=0; i<list.size(); i++)
      delete list[i];
}

bool PLYReader::open(const char *name)
{
    string         line;
    vector<string> format;
    ply_encoding   encoding;
    PLYElement     *element;
    
    filename=name;
    
      // empty list, close file (if necessary) and open new file
    
    for (size_t i=0; i<list.size(); i++)
      delete list[i];
    
    list.clear();
    
    if (in.is_open())
      in.close();
    
    in.exceptions(ios_base::failbit | ios_base::badbit | ios_base::eofbit);
    in.open(name, ios_base::binary);
    
      // check magic code
    
    char id[3];
    id[0]=static_cast<char>(in.get());
    id[1]=static_cast<char>(in.get());
    id[2]=static_cast<char>(in.get());
    
    if (id[0] != 'p' || id[1] != 'l' || id[2] != 'y')
    {
      in.close();
      return false;
    }
    
      // skip rest of the line
    
    getline(in, line);
    
      // read format
    
    getline(in, line);
    trim(line);
    
    split(format, line);
    
    if (format.size() != 3 || format[0].compare("format") != 0 ||
      format[2].compare("1.0") != 0)
      throw IOException("Unknown ply format or version: "+line);
    
    if (format[1].compare("ascii") == 0)
      encoding=ply_ascii;
    else if (format[1].compare("binary_big_endian") == 0)
      encoding=ply_big_endian;
    else if (format[1].compare("binary_little_endian") == 0)
      encoding=ply_little_endian;
    else
      throw IOException("Unknown ply encoding: "+format[1]);
    
      // read all elements and properties of header
    
    getline(in, line);
    trim(line);
    
    while (line.compare("end_header") != 0)
    {
      if (line.compare(0, 7, "element") == 0)
      {
        element=PLYElement::fromString(line);
        list.push_back(element);
      }
      else if (line.compare(0, 7, "comment") == 0)
      {
        list.push_back(PLYElement::fromString(line));
      }
      else if (line.compare(0, 8, "property") == 0)
      {
        if (element == 0)
          throw IOException("Property without an parent element: "+line);
        
        element->addProperty(PLYProperty::fromString(line, encoding));
      }
      
      getline(in, line);
      trim(line);
    }
    
    return true;
}

void PLYReader::printHeader()
{
    for (size_t i=0; i<list.size(); i++)
    {
      PLYElement *element=list[i];
      
      cout << element->toString() << endl;
      
      if (element->getInstances() > 0)
      {
          // write properties
        
        for (int k=0; k<element->getPropertyCount(); k++)
          cout << element->getProperty(k).toString() << endl;
      }
    }
}

void PLYReader::getComments(vector<string> &clist) const
{
    clist.clear();
    
    for (size_t i=0; i<list.size(); i++)
    {
      if (list[i]->getInstances() < 0)
        clist.push_back(list[i]->getName());
    }
}

long PLYReader::instancesOfElement(const string &elem_name) const
{
    PLYElement *elem=findElement(elem_name);
    
    if (elem != 0)
      return elem->getInstances();
    
    return -1;
}

ply_type PLYReader::getTypeOfProperty(const string &elem_name, const string &prop_name) const
{
    PLYElement *elem=findElement(elem_name);
    
    if (elem != 0)
    {
      PLYProperty *prop=elem->findProperty(prop_name);
      
      if (prop != 0)
        return prop->getValueType();
    }
    
    return ply_none;
}
    
bool PLYReader::setReceiver(const string &elem_name, const string &prop_name, PLYReceiver *receiver)
{
    PLYElement *elem=findElement(elem_name);
    
    if (elem != 0)
    {
      PLYProperty *prop=elem->findProperty(prop_name);
      
      if (prop != 0)
      {
        prop->setReceiver(receiver);
        return true;
      }
    }
    
    return false;
}
    
void PLYReader::readData()
{
    if (in.is_open())
    {
      streambuf *sb=in.rdbuf();
      
      for (size_t i=0; i<list.size(); i++)
        list[i]->readData(sb);
      
      in.close();
    }
}

void PLYWriter::writeHeader()
{
    if (!header_complete)
    {
        // write magic code
      
      out << "ply" << endl;
      
        // write encoding and version
      
      out << "format ";
      
      switch (enc)
      {
        case ply_ascii:
            out << "ascii ";
            break;
        
        case ply_big_endian:
            out << "binary_big_endian ";
            break;
        
        case ply_little_endian:
            out << "binary_little_endian ";
            break;
        
        default:
            break;
      }
      
      out << "1.0" << endl;
      
        // write all comments and elements
      
      for (size_t i=0; i<list.size(); i++)
      {
        element=list[i];
        
        out << element->toString() << endl;
        
        if (element->getInstances() > 0)
        {
            // write properties
          
          for (int k=0; k<element->getPropertyCount(); k++)
            out << element->getProperty(k).toString() << endl;
        }
      }
      
      out << "end_header" << "\n";
      
      header_complete=true;
      
        // set element pointer to first element with one or more properties and
        // one or more instances
      
      element=0;
      
      pelem=0;
      while (pelem < list.size() && (list[pelem]->getInstances() <= 0 ||
        list[pelem]->getPropertyCount() == 0))
        pelem++;
      
      pinst=1;
      pprop=1;
      plist=0;
      
      if (pelem < list.size())
      {
        element=list[pelem];
        pinst=element->getInstances();
        pprop=element->getPropertyCount()+1;
      }
    }
}

PLYValue &PLYWriter::nextValue()
{
      // increment internal pointers to the next (list) value, instance or
      // element, according to the specified header
    
    plist--;
    if (plist < 0)
    {
      pprop--;
      if (pprop <= 0)
      {
        pinst--;
        if (pinst <= 0)
        {
          element=0;
          
          pelem++;
          while (pelem < list.size() && (list[pelem]->getInstances() <= 0 ||
            list[pelem]->getPropertyCount() == 0))
            pelem++;
          
          if (pelem >= list.size())
          {
            out.flush();
            throw IOException("Attempt to write more data than specified in the PLY header");
          }
          
          element=list[pelem];
          pinst=element->getInstances();
        }
        
        pprop=element->getPropertyCount();
        
        if (enc == ply_ascii)
          out << endl;
      }
      
      plist=0;
    }
    
    return element->getProperty(element->getPropertyCount()-pprop).getValue();
}

PLYWriter::~PLYWriter()
{
    for (size_t i=0; i<list.size(); i++)
      delete list[i];
}

void PLYWriter::open(const char *name, ply_encoding encoding)
{
      // set encoding
    
    if (encoding == ply_binary)
    {
      if (isMSBFirst())
        encoding=ply_big_endian;
      else
        encoding=ply_little_endian;
    }
    
    enc=encoding;
    
      // empty list, close file (if necessary) and open new file
    
    for (size_t i=0; i<list.size(); i++)
      delete list[i];
    
    list.clear();
    element=0;
    
    header_complete=false;
    
    if (out.is_open())
      out.close();
    
    out.exceptions(ios_base::failbit | ios_base::badbit);
    out.open(name, ios_base::binary);
}

void PLYWriter::addComment(const string &s)
{
    if (header_complete)
      throw IOException("Creation of ply header has been completed");
    
    PLYElement *comment=new PLYElement(s, -1);
    
    list.push_back(comment);
}

void PLYWriter::addElement(const string &name, long instances)
{
    if (header_complete)
      throw IOException("Creation of ply header has been completed");
    
    element=new PLYElement(name, instances);
    
    list.push_back(element);
}

void PLYWriter::addProperty(const string &name, ply_type tvalue)
{
    if (header_complete)
      throw IOException("Creation of ply header has been completed");
    
    if (element == 0)
      throw IOException("PLYWriter: Adding a property without an element");
    
    element->addProperty(PLYProperty(name, tvalue, enc));
}

void PLYWriter::addProperty(const string &name, ply_type tsize, ply_type tvalue)
{
    if (header_complete)
      throw IOException("Creation of ply header has been completed");
    
    if (element == 0)
      throw IOException("PLYWriter: Adding a property without an element");
    
    element->addProperty(PLYProperty(name, tsize, tvalue, enc));
}

void PLYWriter::writeListSize(int v)
{
    if (!header_complete)
      writeHeader();
    
    nextValue().writeListSize(out.rdbuf(), v);
    
    plist=v;
}

void PLYWriter::write(int v)
{
    if (!header_complete)
      writeHeader();
    
    nextValue().write(out.rdbuf(), v);
}

void PLYWriter::write(unsigned int v)
{
    if (!header_complete)
      writeHeader();
    
    nextValue().write(out.rdbuf(), v);
}

void PLYWriter::write(float v)
{
    if (!header_complete)
      writeHeader();
    
    nextValue().write(out.rdbuf(), v);
}

void PLYWriter::write(double v)
{
    if (!header_complete)
      writeHeader();
    
    nextValue().write(out.rdbuf(), v);
}

void PLYWriter::close()
{
    if (!header_complete)
      writeHeader();
    
    if (out.is_open())
    {
      out.close();
      
      pelem++;
      while (pelem < list.size() && (list[pelem]->getInstances() <= 0 ||
        list[pelem]->getPropertyCount() == 0))
        pelem++;
      
      if (!(plist == 0 && pprop == 1 && pinst == 1 && pelem >= list.size()))
        throw IOException("All data as specified in the header must be written. The ply file is corrupted!");
    }
}

}
