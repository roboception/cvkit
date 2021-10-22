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

#include "cameracollection.h"
#include "coloredmesh.h"
#include "coloredlines.h"

#ifdef INCLUDE_GL
#include "glcoloredmesh.h"
#include "glcoloredlines.h"
#include "gltext.h"
#endif

#include <limits>
#include <algorithm>

namespace gvr
{

CameraCollection::CameraCollection(const std::vector<gmath::PinholeCamera> &camlist): cl(camlist)
{ }

void CameraCollection::loadCamera(const char *file)
{
  std::string name=file;

  size_t i=name.find_last_of("/\\");

  if (i != name.npos)
  {
    name=name.substr(i+1);
  }

  if (name == "calib.txt")
  {
    // conversion of pinhole camera parameters from Middlebury format

    gutil::Properties mprop(file), prop;

    gmath::Matrix33d A;
    mprop.getValue("cam0", A);
    prop.putValue("camera.A", A);

    std::string s;
    mprop.getString("width", s, "0");
    prop.putString("camera.width", s);
    mprop.getString("height", s, "0");
    prop.putString("camera.height", s);

    prop.putValue("f", A(0, 0));

    double t;
    mprop.getValue("baseline", t);
    prop.putValue("t", t);

    prop.putString("camera.R", "[1 0 0; 0 1 0; 0 0 1]");
    prop.putValue("camera.T", "[0 0 0]");

    double doffs, dmin, dmax;

    mprop.getValue("doffs", doffs, "0");
    mprop.getValue("vmin", dmin, "0");
    mprop.getValue("vmax", dmax, "0");

    if (dmin > 0 && dmax > 0)
    {
      prop.putValue("camera.zmin", A(0, 0)*t/(dmax+doffs));
      prop.putValue("camera.zmax", A(0, 0)*t/(dmin+doffs));
    }

    prop.putString("camera.match", "cam0, cam1");

    gmath::PinholeCamera cam0(prop);
    cam0.setName("cam0");
    cl.push_back(cam0);

    mprop.getValue("cam1", A);
    prop.putValue("camera.A", A);
    prop.putValue("camera.T", gmath::Vector3d(t, 0, 0));

    gmath::PinholeCamera cam1(prop);
    cam1.setName("cam1");
    cl.push_back(cam1);
  }
  else
  {
    // load text file as pinhole camera and store in list

    gutil::Properties prop(file);
    gmath::PinholeCamera cam(prop);
    cam.setName(name);

    // add camera model

    cl.push_back(cam);
  }
}

void CameraCollection::translate(const gmath::Vector3d &v)
{
  for (size_t i=0; i<cl.size(); i++)
  {
    cl[i].setT(cl[i].getT()+v);
  }

  setDefCameraRT(getDefCameraR(), getDefCameraT()+v);
}

void CameraCollection::addExtend(gmath::Vector3d &emin, gmath::Vector3d &emax) const
{
  for (size_t i=0; i<cl.size(); i++)
  {
    emin[0]=std::min(emin[0], getOrigin()[0]+cl[i].getT()[0]);
    emin[1]=std::min(emin[1], getOrigin()[1]+cl[i].getT()[1]);
    emin[2]=std::min(emin[2], getOrigin()[2]+cl[i].getT()[2]);

    emax[0]=std::max(emax[0], getOrigin()[0]+cl[i].getT()[0]);
    emax[1]=std::max(emax[1], getOrigin()[1]+cl[i].getT()[1]);
    emax[2]=std::max(emax[2], getOrigin()[2]+cl[i].getT()[2]);
  }
}

void CameraCollection::addGLObjects(std::vector<GLObject *> &list)
{
#ifdef INCLUDE_GL
  // compute size of cameras

  double s=0;

  if (cl.size() > 1)
  {
    for (size_t i=0; i<cl.size(); i++)
    {
      double m=std::numeric_limits<double>::max();

      for (size_t k=0; k<cl.size(); k++)
      {
        if (k != i)
        {
          double d=norm(cl[i].getT()-cl[k].getT());

          if (d > 0)
          {
            m=std::min(m, d);
          }
        }
      }

      s+=m;
    }
  }

  if (cl.size() >= 2)
  {
    s/=cl.size();
  }

  s/=3;

  if (s == 0)
  {
    s=0.2;

    if (cl.size() > 0 && cl[0].getZMin() > 0)
    {
      s=cl[0].getZMin()/3;
    }
  }

  // create camera bodies

  {
    ColoredMesh m;

    m.setID(ID_CAMERA_BODY);
    m.setOrigin(getOrigin());

    m.resizeVertexList(5*cl.size(), false, false);
    m.resizeTriangleList(6*cl.size());

    for (size_t i=0; i<cl.size(); i++)
    {
      m.setVertex(5*i, cl[i].getT());
      m.setColor(5*i, 255, 255, 255);

      gmath::Vector2d p[4];
      p[0]=gmath::Vector2d(0, cl[i].getHeight());
      p[1]=gmath::Vector2d(0, 0);
      p[2]=gmath::Vector2d(cl[i].getWidth(), 0);
      p[3]=gmath::Vector2d(cl[i].getWidth(), cl[i].getHeight());

      gmath::Vector3d V, C;

      for (int j=0; j<4; j++)
      {
        cl[i].reconstructRay(V, C, p[j]);

        m.setVertex(5*i+j+1, s*V+C);
        m.setColor(5*i+j+1, 0, 0, 0);

        if (j < 2)
        {
          m.setColorComp(5*i+j+1, 0, 255);
        }
        else
        {
          m.setColorComp(5*i+j+1, 1, 255);
        }
      }

      m.setTriangleIndex(6*i, 5*i+0, 5*i+2, 5*i+1);
      m.setTriangleIndex(6*i+1, 5*i+0, 5*i+3, 5*i+2);
      m.setTriangleIndex(6*i+2, 5*i+0, 5*i+4, 5*i+3);
      m.setTriangleIndex(6*i+3, 5*i+0, 5*i+1, 5*i+4);
      m.setTriangleIndex(6*i+4, 5*i+2, 5*i+3, 5*i+4);
      m.setTriangleIndex(6*i+5, 5*i+2, 5*i+4, 5*i+1);
    }

    m.recalculateNormals();

    list.push_back(new GLColoredMesh(m));
  }

  // create camera names

  {
    GLText *text=new GLText(ID_CAMERA_BODY);

    for (size_t i=0; i<cl.size(); i++)
    {
      gmath::Vector3d V1, V2, C;
      cl[i].reconstructRay(V1, C, gmath::Vector2d(0, 0));
      cl[i].reconstructRay(V2, C, gmath::Vector2d(cl[i].getWidth(), 0));
      text->addText(cl[i].getName(), cl[i].getR(), s*(V1+V2)/2+C, s/8, true);
    }

    list.push_back(text);
  }

  // create z-ranges

  int zn=0;

  for (size_t i=0; i<cl.size(); i++)
  {
    if (cl[i].getZMax() > 0)
    {
      zn++;
    }
  }

  if (zn > 0)
  {
    ColoredLines m;

    m.setID(ID_CAMERA_RANGE);
    m.setOrigin(getOrigin());

    m.resizeVertexList(8*zn, false, false);
    m.resizeLineList(12*zn);

    zn=0;

    for (size_t i=0; i<cl.size(); i++)
    {
      if (cl[i].getZMax() > 0)
      {
        gmath::Vector2d p[4];
        p[0]=gmath::Vector2d(0, cl[i].getHeight());
        p[1]=gmath::Vector2d(0, 0);
        p[2]=gmath::Vector2d(cl[i].getWidth(), 0);
        p[3]=gmath::Vector2d(cl[i].getWidth(), cl[i].getHeight());

        gmath::Vector3d V, C;

        for (int j=0; j<4; j++)
        {
          cl[i].reconstructRay(V, C, p[j]);

          m.setVertex(8*zn+j, cl[i].getZMin()*V+C);
          m.setColor(8*zn+j, 255, 255, 255);

          m.setVertex(8*zn+j+4, cl[i].getZMax()*V+C);
          m.setColor(8*zn+j+4, 255, 255, 255);
        }

        m.setLineIndex(12*zn, 8*zn+0, 8*zn+1);
        m.setLineIndex(12*zn+1, 8*zn+1, 8*zn+2);
        m.setLineIndex(12*zn+2, 8*zn+2, 8*zn+3);
        m.setLineIndex(12*zn+3, 8*zn+3, 8*zn+0);
        m.setLineIndex(12*zn+4, 8*zn+4, 8*zn+5);
        m.setLineIndex(12*zn+5, 8*zn+5, 8*zn+6);
        m.setLineIndex(12*zn+6, 8*zn+6, 8*zn+7);
        m.setLineIndex(12*zn+7, 8*zn+7, 8*zn+4);
        m.setLineIndex(12*zn+8, 8*zn+0, 8*zn+4);
        m.setLineIndex(12*zn+9, 8*zn+1, 8*zn+5);
        m.setLineIndex(12*zn+10, 8*zn+2, 8*zn+6);
        m.setLineIndex(12*zn+11, 8*zn+3, 8*zn+7);

        zn++;
      }
    }

    list.push_back(new GLColoredLines(m));
  }

  // create links between cameras

  {
    std::map<std::string,int> id;
    ColoredLines m;

    m.setID(ID_CAMERA_LINK);
    m.setOrigin(getOrigin());

    m.resizeVertexList(cl.size(), false, false);

    for (size_t i=0; i<cl.size(); i++)
    {
      id.insert(std::pair<std::string,int>(cl[i].getName(), static_cast<int>(i)));
      m.setVertex(i, cl[i].getT());
      m.setColor(i, 255, 255, 0);
    }

    std::vector<unsigned int> link;

    for (size_t i=0; i<cl.size(); i++)
    {
      for (int k=cl[i].countMatch()-1; k>=0; k--)
      {
        std::map<std::string,int>::iterator it=id.find(cl[i].getMatch(k));

        if (it != id.end())
        {
          link.push_back(i);
          link.push_back(it->second);
        }
      }
    }

    m.resizeLineList(link.size()/2);

    for (int i=0; i<m.getLineCount(); i++)
    {
      m.setLineIndex(i, link[(i<<1)], link[(i<<1)+1]);
    }

    list.push_back(new GLColoredLines(m));
  }

#else
  assert(false);
#endif
}

void CameraCollection::loadPLY(PLYReader &ply)
{
  assert(false);
}

void CameraCollection::savePLY(const char *name, bool all, ply_encoding enc) const
{
  assert(false);
}

}
