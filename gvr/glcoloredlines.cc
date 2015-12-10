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

#include "glcoloredlines.h"
#include "glmisc.h"
#include "glcamera.h"

#include "coloredlines.h"

namespace gvr
{

namespace
{

/*
const GLchar *vshader[]=
{
  "#version 130\n",
  
  "in vec3 vertex;",
  "in vec3 vcolor;",
  
  "uniform mat4 trans;",
  
  "out vec3 color;",
  
  "void main()",
  "{",
  "  gl_Position=trans*vec4(vertex, 1.0);",
  "  gl_PointSize=0.0;",
  "  color=vcolor/255;",
  "}",
  0
};

const GLchar *fshader[]=
{
  "#version 130\n",
  
  "in vec3 color;",
  
  "out vec4 FragColor;",
  
  "void main()",
  "{",
  "  FragColor=vec4(color, 1.0);",
  "}",
  0
};
*/

const GLchar *vshader[]=
{
  "#version 120\n",
  
  "attribute vec3 vertex;",
  "attribute vec3 vcolor;",
  
  "uniform mat4 trans;",
  
  "varying vec3 color;",
  
  "void main()",
  "{",
  "  gl_Position=trans*vec4(vertex, 1.0);",
  "  gl_PointSize=0.0;",
  "  color=vcolor/255;",
  "}",
  0
};

const GLchar *fshader[]=
{
  "#version 120\n",
  
  "varying vec3 color;",
  
  "void main()",
  "{",
  "  gl_FragColor=vec4(color, 1.0);",
  "}",
  0
};

}

int   GLColoredLines::init;
GLint GLColoredLines::prg;
GLint GLColoredLines::pvertex;
GLint GLColoredLines::pvcolor;
GLint GLColoredLines::ptrans;

GLColoredLines::GLColoredLines(ColoredLines &p) : GLColoredPointCloud(p)
{
    ln=p.getLineCount();
    
      // create shader programs, but only once per class
    
    if (init == 0)
    {
      prg=createProgram(vshader, fshader);
      
      pvertex=getAttributeLocation(prg, "vertex");
      pvcolor=getAttributeLocation(prg, "vcolor");
      
      ptrans=getUniformLocation(prg, "trans");
    }
    
    init++;
    
      // create buffer objects for data
    
    glGenBuffers(1, &bline);
    glBindBuffer(GL_ARRAY_BUFFER, bline);
    glBufferData(GL_ARRAY_BUFFER, p.getLineCount()*2*sizeof(unsigned int),
      p.getLineArray(), GL_STATIC_DRAW);
    
    checkGLError();
}

GLColoredLines::~GLColoredLines()
{
      // clean up buffer objects
    
    glDeleteBuffers(1, &bline);
    
      // clean up program, but only once per class
    
    init--;
    
    if (init == 0)
    {
      glDeleteProgram(prg);
      prg=0;
    }
}

void GLColoredLines::draw(const GLCamera &cam)
{
    GLint defprg=0;
    glGetIntegerv(GL_CURRENT_PROGRAM, &defprg);
    
    glUseProgram(prg);
    
    glUniformMatrix4fv(ptrans, 1, GL_TRUE, cam.getTransformation());
    
    glEnableVertexAttribArray(pvertex);
    glBindBuffer(GL_ARRAY_BUFFER, bvertex);
    glVertexAttribPointer(pvertex, 3, GL_FLOAT, GL_FALSE, 0, 0);
    
    glEnableVertexAttribArray(pvcolor);
    glBindBuffer(GL_ARRAY_BUFFER, bcolor);
    glVertexAttribPointer(pvcolor, 3, GL_UNSIGNED_BYTE, GL_FALSE, 0, 0);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bline);
    glDrawElements(GL_LINES, 2*ln, GL_UNSIGNED_INT, 0);
    
    glDisableVertexAttribArray(pvcolor);
    glDisableVertexAttribArray(pvertex);
    
    glUseProgram(defprg);
}

}
