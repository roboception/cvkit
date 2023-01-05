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

#include <sstream>
#include <iostream>
#include <algorithm>

namespace gvr
{

void checkGLError()
{
  GLenum err=glGetError();

  if (err == GL_NO_ERROR)
  {
    return;
  }

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
      std::ostringstream out;
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
    std::ostringstream out;
    out << "Error creating shader type: " << type;
    throw GLException(out.str());
  }

  n=0;

  while (src[n] != 0)
  {
    n++;
  }

  glShaderSource(obj, n, src, 0);
  glCompileShader(obj);

  glGetShaderiv(obj, GL_COMPILE_STATUS, &success);

  if (!success)
  {
    GLchar info[1024]="";
    glGetShaderInfoLog(obj, sizeof(info), 0, info);

    std::ostringstream out;
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
  {
    throw GLException("Error creating shader program");
  }

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

    std::ostringstream out;
    out << "Error linking shader program: '" << info << "'";
    throw GLException(out.str());
  }

  return prg;
}

GLint getAttributeLocation(GLuint prg, const GLchar *name)
{
  GLint ret=glGetAttribLocation(prg, name);

  if (ret == -1)
  {
    throw GLException("Cannot find shader program attribute: "+std::string(name));
  }

  return ret;
}

GLint getUniformLocation(GLuint prg, const GLchar *name)
{
  GLint ret=glGetUniformLocation(prg, name);

  if (ret == -1)
  {
    throw GLException("Cannot find shader program parameter: "+std::string(name));
  }

  return ret;
}

}
