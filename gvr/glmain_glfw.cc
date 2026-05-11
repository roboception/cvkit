/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Daniel Scharstein
 *
 * Copyright (c) 2024, 2025
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

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <GLUT/glut.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <chrono>

#include <cstdlib>
#include <cstring>

namespace gvr
{

namespace
{

GLFWwindow *window=0;
GLListener *listener=0;
bool needs_redisplay=false;
int current_modifiers=0;
bool mouse_button_pressed=false;

// stored window position and size for fullscreen toggle
int saved_x=0, saved_y=0, saved_w=800, saved_h=600;

// timer support

struct TimerEntry
{
  double fire_time;
  void (*fct)(int value);
  int value;
};

std::vector<TimerEntry> timers;

bool is_fullscreen=false;

float getContentScale()
{
  int win_w, win_h, fb_w, fb_h;
  glfwGetWindowSize(window, &win_w, &win_h);
  glfwGetFramebufferSize(window, &fb_w, &fb_h);
  return (win_w > 0) ? static_cast<float>(fb_w) / win_w : 1.0f;
}

void getPixelCoords(GLFWwindow *win, double screen_x, double screen_y, int &px, int &py)
{
  int win_w, win_h, fb_w, fb_h;
  glfwGetWindowSize(win, &win_w, &win_h);
  glfwGetFramebufferSize(win, &fb_w, &fb_h);

  float sx = (win_w > 0) ? (float)fb_w / win_w : 1.0f;
  float sy = (win_h > 0) ? (float)fb_h / win_h : 1.0f;

  px = static_cast<int>(screen_x * sx);
  py = static_cast<int>(screen_y * sy);
}

double getCurrentTime()
{
  return glfwGetTime();
}

int mapModifiers(int mods)
{
  int result=0;

  if (mods & GLFW_MOD_SHIFT)
  {
    result|=GLM_MOD_SHIFT;
  }

  if (mods & GLFW_MOD_CONTROL)
  {
    result|=GLM_MOD_CTRL;
  }

  if (mods & GLFW_MOD_ALT)
  {
    result|=GLM_MOD_ALT;
  }

  return result;
}

int mapButton(int glfw_button)
{
  switch (glfw_button)
  {
    case GLFW_MOUSE_BUTTON_LEFT:
      return GLM_BUTTON_LEFT;

    case GLFW_MOUSE_BUTTON_RIGHT:
      return GLM_BUTTON_RIGHT;

    case GLFW_MOUSE_BUTTON_MIDDLE:
      return GLM_BUTTON_MIDDLE;

    default:
      return glfw_button;
  }
}

void onFramebufferSize(GLFWwindow *win, int w, int h)
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

  needs_redisplay=true;
}

void onKey(GLFWwindow *win, int key, int scancode, int action, int mods)
{
  if (action == GLFW_REPEAT)
  {
    return;
  }

  current_modifiers=mapModifiers(mods);

  if (action == GLFW_PRESS)
  {
    // handle special keys that don't generate char callbacks
    if (key == GLFW_KEY_ESCAPE)
    {
      try
      {
        listener->onKey(27, 0, 0);
      }
      catch (const gutil::Exception &ex)
      {
        ex.print();
        exit(10);
      }
    }
    else if (key == GLFW_KEY_TAB)
    {
      try
      {
        listener->onKey('\t', 0, 0);
      }
      catch (const gutil::Exception &ex)
      {
        ex.print();
        exit(10);
      }
    }
  }
}

void onChar(GLFWwindow *win, unsigned int codepoint)
{
  if (codepoint < 128 && codepoint != '\t')
  {
    try
    {
      listener->onKey(static_cast<unsigned char>(codepoint), 0, 0);
    }
    catch (const gutil::Exception &ex)
    {
      ex.print();
      exit(10);
    }
  }
}

void onMouseButton(GLFWwindow *win, int button, int action, int mods)
{
  current_modifiers=mapModifiers(mods);

  int mapped_button=mapButton(button);
  int state=(action == GLFW_PRESS) ? GLM_BUTTON_DOWN : GLM_BUTTON_UP;

  if (action == GLFW_PRESS)
  {
    mouse_button_pressed=true;
  }
  else
  {
    // check if any button is still pressed
    mouse_button_pressed=false;

    if (glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS ||
        glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS ||
        glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS)
    {
      mouse_button_pressed=true;
    }
  }

  double xpos, ypos;
  glfwGetCursorPos(win, &xpos, &ypos);

  int px, py;
  getPixelCoords(win, xpos, ypos, px, py);

  try
  {
    listener->onMouseButton(mapped_button, state, px, py);
  }
  catch (const gutil::Exception &ex)
  {
    ex.print();
    exit(10);
  }
}

void onCursorPos(GLFWwindow *win, double xpos, double ypos)
{
  if (mouse_button_pressed)
  {
    int px, py;
    getPixelCoords(win, xpos, ypos, px, py);

    try
    {
      listener->onMouseMove(px, py);
    }
    catch (const gutil::Exception &ex)
    {
      ex.print();
      exit(10);
    }
  }
}

void onScroll(GLFWwindow *win, double xoffset, double yoffset)
{
  double xpos, ypos;
  glfwGetCursorPos(win, &xpos, &ypos);

  int x, y;
  getPixelCoords(win, xpos, ypos, x, y);

  int mods=0;

  if (glfwGetKey(win, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
      glfwGetKey(win, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS)
  {
    mods|=GLFW_MOD_SHIFT;
  }

  if (glfwGetKey(win, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
      glfwGetKey(win, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS)
  {
    mods|=GLFW_MOD_CONTROL;
  }

  if (glfwGetKey(win, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
      glfwGetKey(win, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS)
  {
    mods|=GLFW_MOD_ALT;
  }

  current_modifiers=mapModifiers(mods);

  try
  {
    if (yoffset > 0)
    {
      listener->onMouseButton(GLM_BUTTON_SCROLL_UP, GLM_BUTTON_DOWN, x, y);
      listener->onMouseButton(GLM_BUTTON_SCROLL_UP, GLM_BUTTON_UP, x, y);
    }
    else if (yoffset < 0)
    {
      listener->onMouseButton(GLM_BUTTON_SCROLL_DOWN, GLM_BUTTON_DOWN, x, y);
      listener->onMouseButton(GLM_BUTTON_SCROLL_DOWN, GLM_BUTTON_UP, x, y);
    }
  }
  catch (const gutil::Exception &ex)
  {
    ex.print();
    exit(10);
  }
}

}

void GLInit(int &argc, char **argv)
{
  if (!glfwInit())
  {
    throw GLException("Failed to initialize GLFW");
  }
}

void GLInitWindow(int x, int y, int w, int h, const char *title)
{
  // request no specific version hints to get GL 2.1 legacy profile on macOS

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

  window=glfwCreateWindow(w, h, title, NULL, NULL);

  if (!window)
  {
    glfwTerminate();
    throw GLException("Failed to create GLFW window");
  }

  glfwSetWindowPos(window, x, y);
  glfwMakeContextCurrent(window);

  // save initial window geometry for fullscreen toggle
  saved_x=x;
  saved_y=y;
  saved_w=w;
  saved_h=h;

  // initialize GLEW

  glewExperimental=GL_TRUE;
  GLenum res=glewInit();

  if (res != GLEW_OK)
  {
    throw GLException(reinterpret_cast<const char *>(glewGetErrorString(res)));
  }

  // clear the GL error generated by glewExperimental
  glGetError();

  glClearColor(0.0f, 0.0f, 0.3f, 0.0f);

  // diagnostics

  std::cerr << "GL Vendor:   " << glGetString(GL_VENDOR) << std::endl;
  std::cerr << "GL Renderer: " << glGetString(GL_RENDERER) << std::endl;
  std::cerr << "GL Version:  " << glGetString(GL_VERSION) << std::endl;
  std::cerr << "GLSL:        " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;

  GLfloat pt_range[2];
  glGetFloatv(GL_POINT_SIZE_RANGE, pt_range);
  std::cerr << "Point size range: " << pt_range[0] << " - " << pt_range[1] << std::endl;

  // settings for drawing

  glEnable(GL_DEPTH_TEST);

  glFrontFace(GL_CCW);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
}

void GLRenderInfoText(const char *p, long fg_rgb, long bg_rgb)
{
  float scale=getContentScale();
  int th=static_cast<int>(15*scale*1.1f);
  void *font=(scale > 1.5f) ? GLUT_BITMAP_TIMES_ROMAN_24 : GLUT_BITMAP_9_BY_15;

  glDisable(GL_DEPTH_TEST);
  glPushMatrix();
  glLoadIdentity();
  GLint size[4];
  glGetIntegerv(GL_VIEWPORT, size);

  glOrtho(0, size[2], size[3], 0, -1, 1);

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

  glPopMatrix();
  glEnable(GL_DEPTH_TEST);
}

void GLRenderInfoLine(const char *p, long fg_rgb, long bg_rgb)
{
  const float stroke_unit=119.05f;
  const float mono_width=104.76f;
  const float char_h=11.0f;
  const float fs=char_h/stroke_unit;
  const float char_w=mono_width*fs;
  const int th=static_cast<int>(char_h+3);

  int win_w, win_h;
  glfwGetWindowSize(window, &win_w, &win_h);

  glDisable(GL_DEPTH_TEST);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, win_w, win_h, 0, -1, 1);

  size_t n=strlen(p);
  int x=0, y=0;
  int w=static_cast<int>(n*char_w)+6;
  int h=th+2;

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

  float lw;
  glGetFloatv(GL_LINE_WIDTH, &lw);
  glLineWidth(getContentScale());

  glPushMatrix();
  glTranslatef(3, th-2, 0);
  glScalef(fs, -fs, 1);

  while (*p != '\0')
  {
    glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, *p++);
  }

  glPopMatrix();

  glLineWidth(lw);
  glPopMatrix();
  glEnable(GL_DEPTH_TEST);
}

void GLRedisplay()
{
  needs_redisplay=true;
}

void GLTimerFunc(unsigned int milliseconds, void (*fct)(int value), int value)
{
  TimerEntry entry;
  entry.fire_time=getCurrentTime()+milliseconds/1000.0;
  entry.fct=fct;
  entry.value=value;
  timers.push_back(entry);
}

void GLMainLoop(GLListener &l)
{
  listener=&l;

  // register callbacks

  glfwSetFramebufferSizeCallback(window, onFramebufferSize);
  glfwSetKeyCallback(window, onKey);
  glfwSetCharCallback(window, onChar);
  glfwSetMouseButtonCallback(window, onMouseButton);
  glfwSetCursorPosCallback(window, onCursorPos);
  glfwSetScrollCallback(window, onScroll);

  // trigger initial reshape

  int fb_w, fb_h;
  glfwGetFramebufferSize(window, &fb_w, &fb_h);
  listener->onReshape(fb_w, fb_h);

  needs_redisplay=true;

  // enter event loop

  while (!glfwWindowShouldClose(window))
  {
    // check timers

    double now=getCurrentTime();

    for (size_t i=0; i<timers.size(); )
    {
      if (now >= timers[i].fire_time)
      {
        void (*fct)(int)=timers[i].fct;
        int val=timers[i].value;
        timers.erase(timers.begin()+i);
        fct(val);
      }
      else
      {
        i++;
      }
    }

    // redraw if needed

    if (needs_redisplay)
    {
      needs_redisplay=false;

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

    // wait for events with a timeout so timers are checked

    if (timers.size() > 0)
    {
      // find minimum fire time
      double min_time=timers[0].fire_time;

      for (size_t i=1; i<timers.size(); i++)
      {
        if (timers[i].fire_time < min_time)
        {
          min_time=timers[i].fire_time;
        }
      }

      double timeout=min_time-getCurrentTime();

      if (timeout > 0)
      {
        glfwWaitEventsTimeout(timeout);
      }
    }
    else
    {
      glfwWaitEvents();
    }
  }

  glfwDestroyWindow(window);
  glfwTerminate();
}

void GLLeaveMainLoop()
{
  if (window)
  {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
}

void GLSwapBuffers()
{
  if (window)
  {
    glfwSwapBuffers(window);
  }
}

int GLGetModifiers()
{
  return current_modifiers;
}

void GLSetWindowSize(int w, int h)
{
  if (window)
  {
    if (is_fullscreen)
    {
      glfwSetWindowMonitor(window, NULL, saved_x, saved_y, w, h, 0);
      is_fullscreen=false;
    }
    else
    {
      glfwSetWindowSize(window, w, h);
    }
  }
}

void GLSetFullscreen(bool enable)
{
  if (!window)
  {
    return;
  }

  if (enable && !is_fullscreen)
  {
    glfwGetWindowPos(window, &saved_x, &saved_y);
    glfwGetWindowSize(window, &saved_w, &saved_h);

    GLFWmonitor *monitor=glfwGetPrimaryMonitor();
    const GLFWvidmode *mode=glfwGetVideoMode(monitor);
    glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
    is_fullscreen=true;
  }
  else if (!enable && is_fullscreen)
  {
    glfwSetWindowMonitor(window, NULL, saved_x, saved_y, saved_w, saved_h, 0);
    is_fullscreen=false;
  }
}

}
