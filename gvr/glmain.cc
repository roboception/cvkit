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

#include <GL/glew.h>

#ifdef __APPLE__
#include <glut.h>
#else
#include <GL/glut.h>
#endif

#include <cstdlib>

using gutil::Exception;

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
      throw GLException(reinterpret_cast<const char *>(glewGetErrorString(res)));
    
      // settings for drawing
    
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
    
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
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
    catch (const Exception &ex)
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
    catch (const Exception &ex)
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
    catch (const Exception &ex)
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
    catch (const Exception &ex)
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
    catch (const Exception &ex)
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
    catch (const Exception &ex)
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

}
