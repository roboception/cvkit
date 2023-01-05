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

#include "glworld.h"
#include "model.h"
#include "globject.h"
#include "glmisc.h"

#include <gimage/image.h>
#include <gimage/io.h>
#include <gmath/linalg.h>
#include <gutil/version.h>

#include <limits>
#include <iostream>
#include <cstdlib>

#include <GL/glew.h>

#ifdef INCLUDE_FLTK
#include <FL/glut.H>
#else
#ifdef __APPLE__
#include <glut.h>
#else
#include <GL/glut.h>
#endif
#endif

#ifdef INCLUDE_PNG
#include <png.h>
#endif

namespace gvr
{

GLWorld::GLWorld(int w, int h, double hfov, const std::string &_keycodes)
{
  keycodes=_keycodes;
  apply_keycodes=(keycodes.size() > 0);

  extmin[0]=std::numeric_limits<float>::max();
  extmin[1]=std::numeric_limits<float>::max();
  extmin[2]=std::numeric_limits<float>::max();

  extmax[0]=-std::numeric_limits<float>::max();
  extmax[1]=-std::numeric_limits<float>::max();
  extmax[2]=-std::numeric_limits<float>::max();

  showid.resize(ID_MODEL_START+10);

  for (size_t i=0; i<showid.size(); i++)
  {
    showid[i]=true;
  }

  showid[ID_CAMERA_BODY]=false;
  showid[ID_CAMERA_RANGE]=false;
  showid[ID_CAMERA_LINK]=false;

  if (hfov > 0)
  {
    camera.setHFoV(hfov);
  }

  camera.setSize(w, h);

  fullscreen=false;

  colorschema=0;

  bg_locked=false;
  glClearColor(0.0f, 0.0f, 0.3f, 0.0f);
  txt_rgb=0xffffff;

  mb=0;
  mx=0;
  my=0;
  mod=0;
}

GLWorld::~GLWorld()
{
  for (size_t i=0; i<list.size(); i++)
  {
    delete list[i];
  }

  list.resize(0);
}

void GLWorld::setBackgroundColor(float red, float green, float blue)
{
  glClearColor(red, green, blue, 0.0f);
  bg_locked=true;
}

void GLWorld::addModel(Model &model)
{
  bool first=(list.size() == 0);

  model.addGLObjects(list);
  model.addExtend(extmin, extmax);

  if (first)
  {
    defRc=model.getDefCameraR();
    defTc=model.getDefCameraT();
    resetCamera();
  }
}

void GLWorld::removeAllModels(int id)
{
  for (size_t i=0; i<list.size(); i++)
  {
    if (list[i]->getID() == id)
    {
      delete list[i];
      list.erase(list.begin()+i);
    }
  }
}

void GLWorld::showCameras(bool show)
{
  showid[ID_CAMERA_BODY]=show;
  showid[ID_CAMERA_LINK]=show;
}

void GLWorld::resetCamera()
{
  gmath::Vector3d center=(extmin+extmax)/2.0;

  double size=extmax[0]-extmin[0];
  size=std::max(size, extmax[1]-extmin[1]);
  size=std::max(size, extmax[2]-extmin[2]);

  camera.init(center, size);

  if (std::abs(det(defRc)-1) < 1e-6)
  {
    camera.setPose(defRc, defTc);
  }
}

void GLWorld::onRedraw()
{
  // before first redraw, apply key codes (except c and q)

  if (apply_keycodes)
  {
    for (size_t k=0; k<keycodes.size(); k++)
    {
      if (keycodes[k] != 'c' && keycodes[k] != 'q')
      {
        onKey(keycodes[k], 0, 0);
      }
    }
  }

  // clear buffer

  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

  // render all objects

  for (size_t i=0; i<list.size(); i++)
  {
    int id=list[i]->getID();

    if (id < 0 || id >= static_cast<int>(showid.size()) || showid[id])
    {
      list[i]->draw(camera);
    }
  }

  if (infotext.size() > 0)
  {
    GLRenderInfoText(infotext.c_str(), 0xffffff, 0x4c4c4c);
  }

  if (infoline.size() > 0)
  {
    GLRenderInfoLine(infoline.c_str(), txt_rgb);
  }

  // after first redraw, apply key codes (only c and q)

  if (apply_keycodes)
  {
    for (size_t k=0; k<keycodes.size(); k++)
    {
      if (keycodes[k] == 'c' || keycodes[k] == 'q')
      {
        onKey(keycodes[k], 0, 0);
      }
    }

    apply_keycodes=false;
  }

  // swap buffer

  glutSwapBuffers();
}

void GLWorld::onReshape(int w, int h)
{
  glViewport(0, 0, w, h);

  camera.setSize(w, h);
}

void GLWorld::onKey(unsigned char key, int x, int y)
{
  bool redisplay=false;

  if (key != 'h' && key != 'v' && infotext.size() > 0)
  {
    infotext.clear();
    redisplay=true;
  }

  if (infoline.size() > 0)
  {
    infoline.clear();
    redisplay=true;
  }

  switch (key)
  {
    case 'h':
      if (infotext.size() == 0)
      {
        std::ostringstream out;

        out << "Usage and Keycodes:\n";
        out << "\n";
        out << "left mouse click and moving for rotation\n";
        out << "left mouse double-click for setting the center of rotation\n";
        out << "shift + left mouse click shows pixel information\n";
        out << "middle mouse click and moving for panning\n";
        out << "right mouse click for zooming (same as mouse wheel)\n";
        out << "\n";
        out << "Information:\n";
        out << "'h'        Shows this help text.\n";
        out << "'v'        Shows version information.\n";
        out << "'i'        Shows information about the 3D model.\n";
        out << "\n";
        out << "Camera and Window:\n";
        out << "'r'        Reset camera position and orientation.\n";
        out << "'f'        Toggle fullscreen on or off.\n";
        out << "'c'        Capture current image to file and print current pose.\n";
        out << "\n";
        out << "Rendering:\n";
        out << "'p'        Toggle between rendering mesh (if available) and point cloud.\n";
        out << "'+', '-'   Increase / decrease point size by 50%.\n";
        out << "'t'        Toggle texture on or off.\n";
        out << "'s'        Toggle between shaded with grey and special colors.\n";
        out << "'b'        Toggle backface culling on or off.\n";
        out << "<tab>      Toggle color schemes.\n";
        out << "\n";
        out << "Showing and Hiding of Objects:\n";
        out << "'0'        Toggle all objects on or off.\n";
        out << "'1' to '9' Toggle object 1 to 9 on or off.\n";
        out << "'k'        Toggle cameras on or off.\n";
        out << "'l'        Toggle links between cameras on or off.\n";
        out << "'z'        Toggle z-range of cameras on or off.\n";
        out << "\n";
        out << "'q', 'esc' Exit.\n";
        infotext=out.str();
      }
      else
      {
        infotext.clear();
      }

      redisplay=true;
      break;

    case 'v':
      if (infotext.size() == 0)
      {
        std::ostringstream out;

        out << "This program is based on cvkit version " << VERSION << "\n";
        out << "Copyright (C) 2016 - 2023 Roboception GmbH\n";
        out << "Copyright (C) 2014, 2015 Institute of Robotics and Mechatronics, German Aerospace Center\n";
        out << "Author: Heiko Hirschmueller\n";
        out << "Contact: heiko.hirschmueller@roboception.de\n";
#ifdef INCLUDE_GDAL
        out << "\n";
        out << "This program is based in part on the Geospatial Data Abstraction Library (GDAL).\n";
#endif
#if defined (INCLUDE_GDAL) || defined (INCLUDE_JPEG)
        out << "\n";
        out << "This program is based in part on the work of the Independent JPEG Group.\n";
#endif
#if defined (INCLUDE_PNG)
        out << png_get_copyright(0) << "\n";
#endif

        infotext=out.str();
      }
      else
      {
        infotext.clear();
      }

      redisplay=true;
      break;

    case 'i':
      {
        int vn=0, tn=0;

        for (size_t i=0; i<list.size(); i++)
        {
          vn+=list[i]->getVertexCount();
          tn+=list[i]->getTriangleCount();
        }

        std::ostringstream out;
        out << "Vertices: " << vn << ", Triangles: " << tn;
        infoline=out.str();
        redisplay=true;
      }
      break;

    case 'r':
      resetCamera();
      redisplay=true;
      break;

    case 'b':
      if (glIsEnabled(GL_CULL_FACE) == GL_TRUE)
      {
        glDisable(GL_CULL_FACE);
      }
      else
      {
        glEnable(GL_CULL_FACE);
      }

      redisplay=true;
      break;

    case '\t':
      if (!bg_locked)
      {
        if (colorschema == 0)
        {
          colorschema=1;

          glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
          txt_rgb=0x4c4c4c;
        }
        else
        {
          colorschema=0;

          glClearColor(0.0f, 0.0f, 0.3f, 0.0f);
          txt_rgb=0xffffff;
        }

        redisplay=true;
      }
      break;

    case 'f':
      if (fullscreen)
      {
        glutReshapeWindow(800, 600);
        redisplay=true;
        fullscreen=false;
      }
      else
      {
        glutFullScreen();
        fullscreen=true;
      }

      break;

    case 'c':
      {
        int w=camera.getWidth();
        int h=camera.getHeight();

        gimage::ImageU8 image(w, h, 3);
        unsigned char *pixel=new unsigned char [w*h];

        // capture image

        glReadPixels(0, 0, w, h, GL_RED, GL_UNSIGNED_BYTE, pixel);

        for (int k=0; k<h; k++)
        {
          for (int i=0; i<w; i++)
          {
            image.set(i, h-k-1, 0, pixel[k*w+i]);
          }
        }

        glReadPixels(0, 0, w, h, GL_GREEN, GL_UNSIGNED_BYTE, pixel);

        for (int k=0; k<h; k++)
        {
          for (int i=0; i<w; i++)
          {
            image.set(i, h-k-1, 1, pixel[k*w+i]);
          }
        }

        glReadPixels(0, 0, w, h, GL_BLUE, GL_UNSIGNED_BYTE, pixel);

        for (int k=0; k<h; k++)
        {
          for (int i=0; i<w; i++)
          {
            image.set(i, h-k-1, 2, pixel[k*w+i]);
          }
        }

        delete [] pixel;

        // store image

        std::string name=gimage::getNewImageName(prefix);

        if (name.size() > 0)
        {
          gimage::getImageIO().save(image, name.c_str());
          infoline="Saved as "+name;
        }
        else
        {
          infoline="Sorry, cannot determine file name for storing image!";
        }

        redisplay=true;
      }
      break;

    case 'q':
    case 27: /* ESC */
      GLLeaveMainLoop();
      return;

    case '0':
      {
        bool all=true;

        for (size_t i=ID_MODEL_START; i<showid.size(); i++)
        {
          all=all && showid[i];
        }

        all=!all;

        for (size_t i=ID_MODEL_START; i<showid.size(); i++)
        {
          showid[i]=all;
        }

        redisplay=true;
      }
      break;

    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      showid[ID_MODEL_START+key-'1']=!showid[ID_MODEL_START+key-'1'];
      redisplay=true;
      break;

    case 'k':
      if (showid[ID_CAMERA_BODY])
      {
        showid[ID_CAMERA_BODY]=false;
        showid[ID_CAMERA_RANGE]=false;
        showid[ID_CAMERA_LINK]=false;
      }
      else
      {
        showid[ID_CAMERA_BODY]=true;
        showid[ID_CAMERA_LINK]=true;
      }

      redisplay=true;
      break;

    case 'z':
      showid[ID_CAMERA_RANGE]=!showid[ID_CAMERA_RANGE];
      redisplay=true;
      break;

    case 'l':
      showid[ID_CAMERA_LINK]=!showid[ID_CAMERA_LINK];
      redisplay=true;
      break;

    default:
      if (camera.onKey(key, x, y))
      {
        redisplay=true;
      }

      break;
  }

  if (redisplay)
  {
    glutPostRedisplay();
  }
}

void GLWorld::onMouseButton(int button, int state, int x, int y)
{
  bool redisplay=false;

  if (state == GLUT_DOWN)
  {
    if (infotext.size() > 0)
    {
      infotext.clear();
      redisplay=true;
    }

    if (infoline.size() > 0)
    {
      infoline.clear();
      redisplay=true;
    }

    mod=glutGetModifiers();

    // check for double click

    mt.stop();

    if (mt.elapsed() < 0.5 && mod == 0 && mb == button &&
        std::abs(static_cast<double>(x-mx)) <= 1 && std::abs(static_cast<double>(y-my)) <= 1)
    {
      mod=GLUT_ACTIVE_CTRL;
    }

    mt.clear();
    mt.start();

    mb=button;
    mx=x;
    my=y;

    if (button == GLUT_LEFT_BUTTON && (mod == GLUT_ACTIVE_SHIFT ||
                                       mod == GLUT_ACTIVE_CTRL))
    {
      // read z-buffer and convert to world coordinate system

      float d;
      glReadPixels(x, camera.getHeight()-y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &d);

      if (d < 1)
      {
        gmath::Vector3d P=camera.pixel2World(x, y, d);
        std::ostringstream out;

        if (mod == GLUT_ACTIVE_CTRL)
        {
          out << "Setting rotation center to: " << offset+P;

          camera.setRotationCenter(P);

          P=transpose(camera.getR())*(P-camera.getT());
          P[2]=0;

          P=camera.getT()+camera.getR()*P;

          camera.setPose(camera.getR(), P);
        }
        else
        {
          out << P;
        }

        infoline=out.str();
        redisplay=true;
      }
    }
  }

  if (camera.onMouseButton(button, state == GLUT_DOWN, x, y))
  {
    redisplay=true;
  }

  if (redisplay)
  {
    glutPostRedisplay();
  }
}

void GLWorld::onMouseMove(int x, int y)
{
  bool redisplay=false;

  if (mb == GLUT_LEFT_BUTTON && mod == GLUT_ACTIVE_SHIFT)
  {
    float d;
    glReadPixels(x, camera.getHeight()-y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &d);

    if (d < 1)
    {
      gmath::Vector3d P=offset+camera.pixel2World(x, y, d);
      std::ostringstream out;

      out << P;

      infoline=out.str();
    }
    else
    {
      infoline.clear();
    }

    redisplay=true;
  }
  else
  {
    if (camera.onMouseMove(x, y))
    {
      redisplay=true;
    }
  }

  if (redisplay)
  {
    glutPostRedisplay();
  }
}

}
