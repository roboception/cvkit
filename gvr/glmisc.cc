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

#include "glmisc.h"

#include <gutil/exception.h>
#include <gutil/misc.h>

#include <sstream>
#include <vector>

#include <iostream>

using std::string;
using std::vector;
using std::ostringstream;
using std::max;

using gutil::split;

namespace gvr
{

void checkGLError()
{
    GLenum err=glGetError();
    
    if (err == GL_NO_ERROR)
      return;
    
    switch (err)
    {
      case GL_NO_ERROR:
          return;
      
      case GL_INVALID_ENUM:
          throw GLException("OpenGL error: Invalid enum");
          
      case GL_INVALID_VALUE:
          throw GLException("OpenGL error: Invalid value");
          
      case GL_INVALID_OPERATION:
          throw GLException("OpenGL error: Invalid operation");
          
      case GL_INVALID_FRAMEBUFFER_OPERATION:
          throw GLException("OpenGL error: Invalid framebuffer operation");
          
      case GL_OUT_OF_MEMORY:
          throw GLException("OpenGL error: Out of memory");
      
      default:
          ostringstream out;
          out << "OpenGL error: Unkown error code: " << err;
          throw GLException(out.str());
    }
}

GLuint createShader(const GLchar **src, GLenum type)
{
    int n;
    GLuint obj=glCreateShader(type);
    GLint success;
    
    if (obj == 0)
    {
      ostringstream out;
      out << "Error creating shader type: " << type;
      throw GLException(out.str());
    }
    
    n=0;
    while (src[n] != 0)
      n++;
    
    glShaderSource(obj, n, src, 0);
    glCompileShader(obj);
    
    glGetShaderiv(obj, GL_COMPILE_STATUS, &success);
    
    if (!success)
    {
      GLchar info[1024]="";
      glGetShaderInfoLog(obj, sizeof(info), 0, info);
      
      ostringstream out;
      out << "Error compiling shader type " << type << ": '" << info << "'";
      throw GLException(out.str());
    }
    
    return obj;
}

GLuint createProgram(const GLchar *vshader[], const GLchar *fshader[])
{
    GLint success=0;
    
    GLuint prg=glCreateProgram();
    
    if (prg == 0)
      throw GLException("Error creating shader program");
    
    GLuint vobj=createShader(vshader, GL_VERTEX_SHADER);
    glAttachShader(prg, vobj);
    
    GLuint fobj=createShader(fshader, GL_FRAGMENT_SHADER);
    glAttachShader(prg, fobj);
    
    glLinkProgram(prg);
    
    glDeleteShader(vobj);
    glDeleteShader(fobj);
    
    glGetProgramiv(prg, GL_LINK_STATUS, &success);
    
    if (!success)
    {
      GLchar info[1024]="";
      glGetProgramInfoLog(prg, sizeof(info), 0, info);
      
      ostringstream out;
      out << "Error linking shader program: '" << info << "'";
      throw GLException(out.str());
    }
    
    return prg;
}

GLint getAttributeLocation(GLuint prg, const GLchar *name)
{
    GLint ret=glGetAttribLocation(prg, name);
    
    if (ret == -1)
      throw GLException("Cannot find shader program attribute: "+string(name));
    
    return ret;
}

GLint getUniformLocation(GLuint prg, const GLchar *name)
{
    GLint ret=glGetUniformLocation(prg, name);
    
    if (ret == -1)
      throw GLException("Cannot find shader program parameter: "+string(name));
    
    return ret;
}

void renderInfoText(const char *p, long fg_rgb, long bg_rgb)
{
    const int th=15;
    void *font=GLUT_BITMAP_9_BY_15;
    
      // prepare for rendering in pixel coordinates
    
    glDisable(GL_DEPTH_TEST);
    glPushMatrix();
    glLoadIdentity();
    GLint size[4];
    glGetIntegerv(GL_VIEWPORT, size);
    
    glOrtho(0, size[2], size[3], 0, -1, 1);
    
      // determine width and height
    
    vector<string> list;
    
    split(list, string(p), '\n', false);
    
    int x=0, y=0;
    int w=0, h=0;
    
    for (size_t i=0; i<list.size(); i++)
    {
      if (list[i].size() > 0)
      {
        int s=0;
        
        for (size_t k=0; k<list[i].size(); k++)
          s+=glutBitmapWidth(font, list[i][k]);
        
        w=max(w, s);
      }
      
      h+=th;
    }
    
    w+=4;
    h+=4;
    
    if (size[2] > w)
      x+=(size[2]-w)/2;
    
    if (size[3] > h)
      y+=(size[3]-h)/2;
    
      // render background and text
    
    glColor3f(((bg_rgb>>16)&0xff)/255.0f, ((bg_rgb>>8)&0xff)/255.0f,
      (bg_rgb&0xff)/255.0f);
    
    glBegin(GL_POLYGON);
    glVertex2i(x, y);
    glVertex2i(x, y+h);
    glVertex2i(x+w, y+h);
    glVertex2i(x+w, y);
    glEnd();
    
    glColor3f(((fg_rgb>>16)&0xff)/255.0f, ((fg_rgb>>8)&0xff)/255.0f,
      (fg_rgb&0xff)/255.0f);
    
    for (size_t i=0; i<list.size(); i++)
    {
      if (list[i].size() > 0)
      {
        glRasterPos2i(x+2, y+(i+1)*th);
        
        for (size_t k=0; k<list[i].size(); k++)
          glutBitmapCharacter(font, list[i][k]);
      }
    }
    
      // reset settings
    
    glPopMatrix();
    glEnable(GL_DEPTH_TEST);
}

void renderInfoLine(const char *p, long fg_rgb)
{
    void *font=GLUT_BITMAP_9_BY_15;
    
      // prepare for rendering in pixel coordinates
    
    glDisable(GL_DEPTH_TEST);
    glPushMatrix();
    glLoadIdentity();
    
    GLint size[4];
    glGetIntegerv(GL_VIEWPORT, size);
    
    glOrtho(0, size[2], size[3], 0, -1, 1);
    
      // render text
    
    glColor3f(((fg_rgb>>16)&0xff)/255.0f, ((fg_rgb>>8)&0xff)/255.0f,
      (fg_rgb&0xff)/255.0f);
    
    glRasterPos2i(4, size[3]-4);
    
    while (*p != '\0')
      glutBitmapCharacter(font, *p++);
    
      // reset settings
    
    glPopMatrix();
    glEnable(GL_DEPTH_TEST);
}

}
