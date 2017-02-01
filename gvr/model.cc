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

#include "model.h"

#include "pointcloud.h"
#include "coloredpointcloud.h"
#include "mesh.h"
#include "coloredmesh.h"
#include "texturedmesh.h"
#include "multitexturedmesh.h"

#include <gimage/io.h>
#include <gimage/view.h>
#include <gutil/misc.h>
#include <gmath/linalg.h>

#include <set>

namespace gvr
{

void Model::setOriginFromPLY(PLYReader &ply)
{
  std::vector<std::string> comment;

  ply.getComments(comment);

  for (size_t i=0; i<comment.size(); i++)
  {
    std::istringstream in(comment[i]);
    std::string s;

    in >> s;

    if (s.compare("Origin") == 0 || s.compare("origin") == 0)
    {
      in >> origin[0] >> origin[1] >> origin[2];

      if (in.fail())
      {
        origin[0]=origin[1]=origin[2]=0;
      }
    }

    if (s.compare("Camera") == 0 || s.compare("camera") == 0)
    {
      try
      {
        in >> Rc >> Tc;
      }
      catch (const gutil::Exception &ex)
      {
        Rc=0;
        Tc=0;
      }
    }
  }
}

void Model::setOriginToPLY(PLYWriter &ply) const
{
  std::ostringstream out;

  out.precision(16);
  out << "Origin " << origin[0] << " " << origin[1] << " " << origin[2];
  ply.addComment(out.str());

  if (std::abs(det(Rc)-1) < 1e-6)
  {
    std::ostringstream out2;

    out2 << "Camera " << Rc << " " << Tc;
    ply.addComment(out2.str());
  }
}

FloatArrayReceiver::FloatArrayReceiver(float *_array, size_t _size, size_t _offset)
{
  array=_array;
  size=_size;
  offset=_offset;
}

void FloatArrayReceiver::setValue(int instance, const PLYValue &value)
{
  array[instance*size+offset]=value.getFloat();
}

UInt8Receiver::UInt8Receiver(unsigned char *_array, int _offset, float _scale)
{
  array=_array;
  offset=_offset;
  scale=_scale;
}

void UInt8Receiver::setValue(int instance, const PLYValue &value)
{
  array[3*instance+offset]=static_cast<unsigned char>(scale*value.getFloat());
}

Model *loadModel(const char *name, const char *spath, bool verbose)
{
  std::string s=name;

  if (s.rfind(".ply") == s.size()-4 || s.rfind(".PLY") == s.size()-4)
  {
    return loadPLY(name);
  }

  return loadDepth(name, spath, verbose);
}

Model *loadPLY(const char *name)
{
  PLYReader ply;

  ply.open(name);

  // determine data type

  int vn=static_cast<int>(ply.instancesOfElement("vertex"));
  int tn=static_cast<int>(ply.instancesOfElement("face"));

  bool pervertexcolor=ply.getTypeOfProperty("vertex", "diffuse_red") != ply_none ||
                      ply.getTypeOfProperty("vertex", "red") != ply_none;

  bool texture=(ply.getTypeOfProperty("vertex", "u") != ply_none &&
                ply.getTypeOfProperty("vertex", "v") != ply_none) ||
               (ply.getTypeOfProperty("vertex", "texture_u") != ply_none &&
                ply.getTypeOfProperty("vertex", "texture_v") != ply_none) ||
               (ply.getTypeOfProperty("vertex", "s") != ply_none &&
                ply.getTypeOfProperty("vertex", "t") != ply_none);

  bool multitexture=ply.getTypeOfProperty("face", "texcoord") != ply_none &&
                    ply.getTypeOfProperty("face", "texnumber") != ply_none;

  // create best data type

  Model *model=0;
  TexturedMesh *tm=0;

  if (vn > 0)
  {
    if (tn > 0)
    {
      if (multitexture)
      {
        model=new MultiTexturedMesh();
      }
      else if (texture)
      {
        model=tm=new TexturedMesh();
      }
      else if (pervertexcolor)
      {
        model=new ColoredMesh();
      }
      else
      {
        model=new Mesh();
      }
    }
    else
    {
      if (pervertexcolor)
      {
        model=new ColoredPointCloud();
      }
      else
      {
        model=new PointCloud();
      }
    }

    // load from PLY into the allocated data set

    model->loadPLY(ply);
  }

  // optionally try to guess the name of the texture from the file name

  if (tm != 0 && tm->getTextureName().size() == 0)
  {
    std::string s=name;
    size_t i=s.rfind('.');

    if (i < s.size())
    {
      std::set<std::string> list;
      gutil::getFileList(list, s.substr(0, i+1), "");

      for (std::set<std::string>::iterator it=list.begin(); it!=list.end(); ++it)
      {
        if (s.compare(*it) != 0  && gimage::getImageIO().handlesFile(it->c_str(), true))
        {
          s=*it;

          i=s.rfind('/');

          if (i == s.npos)
          {
            i=s.rfind('\\');
          }

          if (i != s.npos)
          {
            tm->setBasePath(s.substr(0, i+1));
            tm->setTextureName(s.substr(i+1));
          }
          else
          {
            tm->setBasePath("");
            tm->setTextureName(s);
          }

          break;
        }
      }
    }
  }

  return model;
}

Model *loadDepth(const char *name, const char *spath, bool verbose)
{
  // read optional maximum step size parameter

  float dstep=1.0f;

  std::vector<std::string> list;
  gutil::split(list, name, ',');

  for (size_t i=1; i<list.size(); i++)
  {
    if (list[i].compare(0, 2, "s=") == 0)
    {
      dstep=atof(list[i].substr(2).c_str());
    }
  }

  // load view

  gimage::View view;
  loadView(view, name, spath, verbose);

  // determine the number of valid points

  const gimage::ImageFloat &depth=view.getDepthImage();

  int n=0;

  for (long k=0; k<depth.getHeight(); k++)
  {
    for (long i=0; i<depth.getWidth(); i++)
    {
      if (depth.isValid(i, k))
      {
        n++;
      }
    }
  }

  // create mesh

  Mesh *mesh=0;
  const gimage::ImageU8 &image=view.getImage();

  if (image.getWidth() > 0 && image.getHeight() > 0)
  {
    ColoredMesh *cmesh=new ColoredMesh();
    cmesh->resizeVertexList(n, true, true);

    // store colors

    n=0;

    for (long k=0; k<image.getHeight(); k++)
    {
      for (long i=0; i<image.getWidth(); i++)
      {
        if (depth.isValid(i, k))
        {
          if (image.getDepth() == 3)
          {
            cmesh->setColorComp(n, 0, image.get(i, k, 0));
            cmesh->setColorComp(n, 1, image.get(i, k, 1));
            cmesh->setColorComp(n, 2, image.get(i, k, 2));
          }
          else
          {
            cmesh->setColorComp(n, 0, image.get(i, k, 0));
            cmesh->setColorComp(n, 1, image.get(i, k, 0));
            cmesh->setColorComp(n, 2, image.get(i, k, 0));
          }

          n++;
        }
      }
    }

    mesh=cmesh;
  }
  else
  {
    mesh=new Mesh();
    mesh->resizeVertexList(n, true, true);
  }

  // reconstruct and store vertices

  const gmath::Camera *cam=view.getCamera();

  mesh->setOrigin(cam->getT());

  if (cam->isPerspective())
  {
    mesh->setDefCameraRT(cam->getR(), gmath::Vector3d());
  }

  n=0;

  for (long k=0; k<depth.getHeight(); k++)
  {
    for (long i=0; i<depth.getWidth(); i++)
    {
      if (depth.isValid(i, k))
      {
        gmath::Vector2d p(i+0.5, k+0.5);
        double   d=depth.get(i, k);
        gmath::Vector3d P, P2;

        cam->reconstructPoint(P, p, d);

        mesh->setVertexComp(n, 0, static_cast<float>(P[0]-cam->getT()[0]));
        mesh->setVertexComp(n, 1, static_cast<float>(P[1]-cam->getT()[1]));
        mesh->setVertexComp(n, 2, static_cast<float>(P[2]-cam->getT()[2]));

        p[0]+=0.5;
        p[1]+=0.5;

        cam->reconstructPoint(P2, p, d);
        mesh->setScanSize(n, static_cast<float>(2*norm(P2-P)));

        p[0]-=0.5;
        p[1]-=0.5;
        d+=0.5;

        cam->reconstructPoint(P2, p, d);
        mesh->setScanError(n, static_cast<float>(norm(P2-P)));
        mesh->setScanConf(n, 1.0f);

        mesh->setScanPosComp(n, 0, 0.0f);
        mesh->setScanPosComp(n, 1, 0.0f);
        mesh->setScanPosComp(n, 2, 0.0f);

        n++;
      }
    }
  }

  // count number of triangles

  int tn=0;

  for (long k=1; k<depth.getHeight(); k++)
  {
    for (long i=1; i<depth.getWidth(); i++)
    {
      float dmin=std::numeric_limits<float>::max();
      float dmax=-std::numeric_limits<float>::max();
      int   valid=0;

      for (int kk=0; kk<2; kk++)
      {
        for (int ii=0; ii<2; ii++)
        {
          if (depth.isValid(i-ii, k-kk))
          {
            dmin=std::min(dmin, depth.get(i-ii, k-kk));
            dmax=std::max(dmax, depth.get(i-ii, k-kk));
            valid++;
          }
        }
      }

      if (valid >= 3 && dmax-dmin <= dstep)
      {
        tn+=valid-2;
      }
    }
  }

  mesh->resizeTriangleList(tn);

  // create triangles

  std::vector<int> line0(depth.getWidth());
  std::vector<int> line1(depth.getWidth());
  std::vector<int> *l0=&line0;
  std::vector<int> *l1=&line1;

  n=0;
  tn=0;

  for (long i=0; i<depth.getWidth(); i++)
  {
    l1->at(i)=-1;

    if (depth.isValid(i, 0))
    {
      l1->at(i)=n++;
    }
  }

  for (long k=1; k<depth.getHeight(); k++)
  {
    std::vector<int> *t=l0;
    l0=l1;
    l1=t;

    l1->at(0)=-1;

    if (depth.isValid(0, k))
    {
      l1->at(0)=n++;
    }

    for (long i=1; i<depth.getWidth(); i++)
    {
      float dmin=std::numeric_limits<float>::max();
      float dmax=-std::numeric_limits<float>::max();
      int   valid=0;

      l1->at(i)=-1;

      if (depth.isValid(i, k))
      {
        l1->at(i)=n++;
      }

      for (int kk=0; kk<2; kk++)
      {
        for (int ii=0; ii<2; ii++)
        {
          if (depth.isValid(i-ii, k-kk))
          {
            dmin=std::min(dmin, depth.get(i-ii, k-kk));
            dmax=std::max(dmax, depth.get(i-ii, k-kk));
            valid++;
          }
        }
      }

      if (valid >= 3 && dmax-dmin <= dstep)
      {
        int j=0;
        int f[4];

        if (l0->at(i-1) >= 0)
        {
          f[j++]=l0->at(i-1);
        }

        if (l1->at(i-1) >= 0)
        {
          f[j++]=l1->at(i-1);
        }

        if (l1->at(i) >= 0)
        {
          f[j++]=l1->at(i);
        }

        if (l0->at(i) >= 0)
        {
          f[j++]=l0->at(i);
        }

        mesh->setTriangleIndex(tn, 0, f[0]);
        mesh->setTriangleIndex(tn, 1, f[1]);
        mesh->setTriangleIndex(tn, 2, f[2]);
        tn++;

        if (j == 4)
        {
          mesh->setTriangleIndex(tn, 0, f[2]);
          mesh->setTriangleIndex(tn, 1, f[3]);
          mesh->setTriangleIndex(tn, 2, f[0]);
          tn++;
        }
      }
    }
  }

  // compute normals

  mesh->recalculateNormals();

  return mesh;
}

}