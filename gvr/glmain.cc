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

#include "glmain.h"
#include "glmisc.h"

#include <gutil/misc.h>

#include <GL/glew.h>

#ifdef INCLUDE_FLTK
#include <FL/glut.H>
#else
#ifdef __APPLE__
#include <glut.h>
#else
#include <GL/freeglut.h>
#define USES_FREEGLUT
#endif
#endif

#include <sstream>
#include <vector>

#include <cstdlib>

namespace gvr
{

void GLInit(int &argc, char **argv)
{
  glutInit(&argc, argv);
}

void GLInitWindow(int x, int y, int w, int h, const char *title)
{
  // create GLUT window

  glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA|GLUT_DEPTH);
  glutInitWindowSize(w, h);
  glutInitWindowPosition(x, y);
  glutCreateWindow(title);

  glClearColor(0.0f, 0.0f, 0.3f, 0.0f);

  // initialize extensions

  GLenum res=glewInit();

  if (res != GLEW_OK)
  {
    throw GLException(reinterpret_cast<const char *>(glewGetErrorString(res)));
  }

  // settings for drawing

  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);

  glFrontFace(GL_CCW);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
}

void GLRenderInfoText(const char *p, long fg_rgb, long bg_rgb)
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

  std::vector<std::string> list;

  gutil::split(list, std::string(p), '\n', false);

  int x=0, y=0;
  int w=0, h=0;

  for (size_t i=0; i<list.size(); i++)
  {
    if (list[i].size() > 0)
    {
      int s=0;

      for (size_t k=0; k<list[i].size(); k++)
      {
        s+=glutBitmapWidth(font, list[i][k]);
      }

      w=std::max(w, s);
    }

    h+=th;
  }

  w+=4;
  h+=4;

  if (size[2] > w)
  {
    x+=(size[2]-w)/2;
  }

  if (size[3] > h)
  {
    y+=(size[3]-h)/2;
  }

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
      glRasterPos2i(x+2, y+static_cast<int>(i+1)*th);

      for (size_t k=0; k<list[i].size(); k++)
      {
        glutBitmapCharacter(font, list[i][k]);
      }
    }
  }

  // reset settings

  glPopMatrix();
  glEnable(GL_DEPTH_TEST);
}

void GLRenderInfoLine(const char *p, long fg_rgb)
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
  {
    glutBitmapCharacter(font, *p++);
  }

  // reset settings

  glPopMatrix();
  glEnable(GL_DEPTH_TEST);
}

void GLRedisplay()
{
  glutPostRedisplay();
}

void GLTimerFunc(unsigned int milliseconds, void (*fct)(int value), int value)
{
  glutTimerFunc(milliseconds, fct, value);
}

namespace
{

GLListener *listener=0;

void onRedraw()
{
  try
  {
    listener->onRedraw();
  }
  catch (const gutil::Exception &ex)
  {
    ex.print();
    exit(10);
  }
}

void onReshape(int w, int h)
{
  try
  {
    listener->onReshape(w, h);
  }
  catch (const gutil::Exception &ex)
  {
    ex.print();
    exit(10);
  }
}

void onSpecialKey(int key, int x, int y)
{
  try
  {
    listener->onSpecialKey(key, x, y);
  }
  catch (const gutil::Exception &ex)
  {
    ex.print();
    exit(10);
  }
}

void onKey(unsigned char key, int x, int y)
{
  try
  {
    listener->onKey(key, x, y);
  }
  catch (const gutil::Exception &ex)
  {
    ex.print();
    exit(10);
  }
}

void onMouseMove(int x, int y)
{
  try
  {
    listener->onMouseMove(x, y);
  }
  catch (const gutil::Exception &ex)
  {
    ex.print();
    exit(10);
  }
}

void onMouseButton(int button, int state, int x, int y)
{
  try
  {
    listener->onMouseButton(button, state, x, y);
  }
  catch (const gutil::Exception &ex)
  {
    ex.print();
    exit(10);
  }
}

}

void GLMainLoop(GLListener &l)
{
  listener=&l;

  // register callbacks

  glutDisplayFunc(onRedraw);
  glutReshapeFunc(onReshape);
  glutSpecialFunc(onSpecialKey);
  glutKeyboardFunc(onKey);
  glutMouseFunc(onMouseButton);
  glutMotionFunc(onMouseMove);

  // enter event loop

  glutMainLoop();
}

/**
  Leaves main loop.
*/

void GLLeaveMainLoop()
{
#ifdef USES_FREEGLUT
    glutLeaveMainLoop(); // clean way to exit loop/program
#else
    exit(0);
#endif
}

}
