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

#include "gltexturedmesh.h"
#include "glmisc.h"
#include "glcamera.h"

#include "texturedmesh.h"

#include <gimage/image.h>
#include <gimage/io.h>

namespace gvr
{

namespace
{

/*

const GLchar *vshader[]=
{
  "#version 130\n",

  "in vec3 vertex;",
  "in float size;",
  "in vec2 vtex;",

  "uniform mat4 trans;",
  "uniform float f;",

  "out vec2 tex;",

  "void main()",
  "{",
  "  gl_Position=trans*vec4(vertex, 1.0);",
  "  gl_PointSize=max(1.0, f*size/gl_Position.w);",
  "  tex=vtex;",
  "}",
  0
};

const GLchar *fshader[]=
{
  "#version 130\n",

  "in vec2 tex;",

  "uniform bool grey;",
  "uniform sampler2D id;",

  "out vec4 FragColor;",

  "void main()",
  "{",
  "  FragColor=texture2D(id, vec2(tex.x, 1-tex.y));",
  "  if (grey) { FragColor.y=FragColor.x; FragColor.z=FragColor.x; }",
  "}",
  0
};

*/

#ifdef __APPLE__

const GLchar *vshader[]=
{
  "#version 120\n",

  "attribute vec3 vertex;",
  "attribute vec2 vtex;",

  "uniform mat4 trans;",

  "varying vec2 tex;",

  "void main()",
  "{",
  "  gl_Position=trans*vec4(vertex, 1.0);",
  "  tex=vtex;",
  "}",
  0
};

#else

const GLchar *vshader[]=
{
  "#version 120\n",

  "attribute vec3 vertex;",
  "attribute float size;",
  "attribute vec2 vtex;",

  "uniform mat4 trans;",
  "uniform float f;",

  "varying vec2 tex;",

  "void main()",
  "{",
  "  gl_Position=trans*vec4(vertex, 1.0);",
  "  gl_PointSize=max(1.0, f*size/gl_Position.w);",
  "  tex=vtex;",
  "}",
  0
};

#endif

const GLchar *fshader[]=
{
  "#version 120\n",

  "varying vec2 tex;",

  "uniform bool grey;",
  "uniform sampler2D id;",

  "void main()",
  "{",
  "  gl_FragColor=texture2D(id, vec2(tex.x, 1-tex.y));",
  "  if (grey) { gl_FragColor.y=gl_FragColor.x; gl_FragColor.z=gl_FragColor.x; }",
  "}",
  0
};

}

int   GLTexturedMesh::init;
GLint GLTexturedMesh::prg;
GLint GLTexturedMesh::pvertex;
GLint GLTexturedMesh::pvtex;
GLint GLTexturedMesh::ptrans;
GLint GLTexturedMesh::pgrey;
GLint GLTexturedMesh::pid;

#ifndef __APPLE__
GLint GLTexturedMesh::psize;
GLint GLTexturedMesh::pf;
#endif

GLTexturedMesh::GLTexturedMesh(TexturedMesh &p) : GLMesh(p)
{
  // create shader programs, but only once per class

  if (init == 0)
  {
    prg=createProgram(vshader, fshader);

    pvertex=getAttributeLocation(prg, "vertex");
    pvtex=getAttributeLocation(prg, "vtex");

    ptrans=getUniformLocation(prg, "trans");
    pgrey=getUniformLocation(prg, "grey");
    pid=getUniformLocation(prg, "id");

#ifndef __APPLE__
    psize=getAttributeLocation(prg, "size");
    pf=getUniformLocation(prg, "f");
#endif
  }

  init++;

  // create buffer objects for data

  glGenBuffers(1, &buv);
  glBindBuffer(GL_ARRAY_BUFFER, buv);
  glBufferData(GL_ARRAY_BUFFER, p.getVertexCount()*2*sizeof(float),
               p.getTextureCoordArray(), GL_STATIC_DRAW);

  GLint maxsize;
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxsize);

  grey=0;
  btext=0;

  try
  {
    long w, h;
    int  d, ds;

    // determine if image needs to be loaded smaller

    gimage::getImageIO().loadHeader((p.getBasePath()+p.getTextureName()).c_str(), w, h, d);

    ds=1;

    while ((w+ds-1)/ds > maxsize || (h+ds-1)/ds > maxsize)
    {
      ds++;
    }

    gimage::ImageU8 image;

    gimage::getImageIO().load(image, (p.getBasePath()+p.getTextureName()).c_str(), ds);

    if (image.getDepth() == 3) // convert color texture
    {
      grey=0;

      if (((3*image.getWidth()) & 0x3) != 0)
      {
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
      }

      glGenTextures(1, &btext);
      glBindTexture(GL_TEXTURE_2D, btext);

      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

      GLubyte *pixel=new GLubyte [3*image.getWidth()*image.getHeight()];

      size_t j=0;

      for (long k=0; k<image.getHeight(); k++)
      {
        for (long i=0; i<image.getWidth(); i++)
        {
          pixel[j++]=image.get(i, k, 0);
          pixel[j++]=image.get(i, k, 1);
          pixel[j++]=image.get(i, k, 2);
        }
      }

      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.getWidth(),
                   image.getHeight(), 0, GL_RGB, GL_UNSIGNED_BYTE, pixel);

      delete [] pixel;
    }
    else // convert greyscale texture
    {
      grey=1;

      if ((image.getWidth() & 0x3) != 0)
      {
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
      }

      glGenTextures(1, &btext);
      glBindTexture(GL_TEXTURE_2D, btext);

      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

      GLubyte *pixel=new GLubyte [image.getWidth()*image.getHeight()];

      size_t j=0;

      for (long k=0; k<image.getHeight(); k++)
        for (long i=0; i<image.getWidth(); i++)
        {
          pixel[j++]=image.get(i, k);
        }

      glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, image.getWidth(),
                   image.getHeight(), 0, GL_RED, GL_UNSIGNED_BYTE, pixel);
    }
  }
  catch (gutil::Exception &ex)
  {
    std::cerr << ex.what() << std::endl;
  }

  checkGLError();
}

GLTexturedMesh::~GLTexturedMesh()
{
  // clean up buffer objects

  glDeleteTextures(1, &btext);
  glDeleteBuffers(1, &buv);

  // clean up program, but only once per class

  init--;

  if (init == 0)
  {
    glDeleteProgram(prg);
    prg=0;
  }
}

void GLTexturedMesh::draw(const GLCamera &cam)
{
  if (!cam.getRenderTexture() || btext == 0)
  {
    GLMesh::draw(cam);
    return;
  }

  GLint defprg=0;
  glGetIntegerv(GL_CURRENT_PROGRAM, &defprg);

  glUseProgram(prg);

  glUniformMatrix4fv(ptrans, 1, GL_TRUE, cam.getTransformation());

#ifndef __APPLE__
  double ps=cam.getPointScale();

  if (bsize != 0 && ps == 0)
  {
    ps=1;
  }

  glUniform1f(pf, static_cast<GLfloat>(cam.getFocalLength()*ps));
#endif

  glUniform1i(pid, 0);
  glUniform1i(pgrey, grey);

  glEnableVertexAttribArray(pvertex);
  glBindBuffer(GL_ARRAY_BUFFER, bvertex);
  glVertexAttribPointer(pvertex, 3, GL_FLOAT, GL_FALSE, 0, 0);

  glEnableVertexAttribArray(pvtex);
  glBindBuffer(GL_ARRAY_BUFFER, buv);
  glVertexAttribPointer(pvtex, 2, GL_FLOAT, GL_FALSE, 0, 0);

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, btext);

#ifndef __APPLE__

  if (bsize != 0 && cam.getRenderPointsOnly())
  {
    glEnableVertexAttribArray(psize);
    glBindBuffer(GL_ARRAY_BUFFER, bsize);
    glVertexAttribPointer(psize, 1, GL_FLOAT, GL_FALSE, 0, 0);
  }
  else
  {
    glVertexAttrib1f(psize, 1);
  }

#endif

  if (!cam.getRenderPointsOnly())
  {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, btriangle);
    glDrawElements(GL_TRIANGLES, 3*tn, GL_UNSIGNED_INT, 0);
  }
  else
  {
    glDrawArrays(GL_POINTS, 0, vn);
  }

#ifndef __APPLE__

  if (bsize != 0 && cam.getRenderPointsOnly())
  {
    glDisableVertexAttribArray(psize);
  }

#endif

  glDisableVertexAttribArray(pvtex);
  glDisableVertexAttribArray(pvertex);

  glUseProgram(defprg);
}

}