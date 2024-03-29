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
#include "extrusion.h"

#include <gutil/misc.h>
#include <gmath/linalg.h>
#include <gmath/pose.h>
#include <gimage/io.h>
#include <gimage/size.h>
#include <gimage/view.h>
#include <gimage/polygon.h>

#include <set>
#include <fstream>
#include <sstream>

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
      catch (const gutil::Exception &)
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

Model *loadModel(const char *name, const char *spath, bool verbose, bool merge_double_vertices)
{
  std::string s=name;

  if (s.rfind(".stl") == s.size()-4 || s.rfind(".STL") == s.size()-4)
  {
    return loadSTL(name, merge_double_vertices);
  }
  else if (s.rfind(".ply") == s.size()-4 || s.rfind(".PLY") == s.size()-4)
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
                      ply.getTypeOfProperty("vertex", "red") != ply_none ||
                      ply.getTypeOfProperty("vertex", "r") != ply_none;

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

namespace
{

inline uint32_t stlReadUint32(std::ifstream &in)
{
  uint32_t ret;

  ret=static_cast<uint32_t>(in.get());
  ret|=static_cast<uint32_t>(in.get())<<8;
  ret|=static_cast<uint32_t>(in.get())<<16;
  ret|=static_cast<uint32_t>(in.get())<<24;

  return ret;
}

inline float stlReadFloat32(std::ifstream &in)
{
  uint32_t vi=stlReadUint32(in);
  float ret;

  memcpy(reinterpret_cast<char *>(&ret), reinterpret_cast<char *>(&vi), 4);

  return ret;
}

inline gmath::Vector3f stlReadVector3f(std::ifstream &in)
{
  gmath::Vector3f ret;

  ret[0]=stlReadFloat32(in);
  ret[1]=stlReadFloat32(in);
  ret[2]=stlReadFloat32(in);

  return ret;
}

struct VertexNormalIndex
{
  VertexNormalIndex(const gmath::Vector3f &v, const gmath::Vector3f &tnormal, uint32_t _index)
  {
    x=v[0];
    y=v[1];
    z=v[2];
    nx=tnormal[0];
    ny=tnormal[1];
    nz=tnormal[2];
    index=_index;
  }

  float x, y, z;
  float nx, ny, nz;
  uint32_t index;
};

bool operator<(const VertexNormalIndex &a, const VertexNormalIndex &b)
{
  float eps=0.1f;

  if (a.x < b.x)
  {
    return true;
  }
  else if (a.x == b.x)
  {
    if (a.y < b.y)
    {
      return true;
    }
    else if (a.y == b.y)
    {
      if (a.z < b.z)
      {
        return true;
      }
      else if (a.z == b.z)
      {
        if (a.nx < b.nx-eps)
        {
          return true;
        }
        else if (a.nx < b.nx+eps)
        {
          if (a.ny < b.ny-eps)
          {
            return true;
          }
          else if (a.ny < b.ny+eps)
          {
            if (a.nz < b.nz-eps)
            {
              return true;
            }
          }
        }
      }
    }
  }

  return false;
}

unsigned int findOrStoreVertex(Mesh *mesh, std::set<VertexNormalIndex> &vni,
  int &vn, const gmath::Vector3f &v, const gmath::Vector3f &normal)
{
  // try to find point v in the vertex list of mesh, but it is only considered
  // a duplicate if the normals are the same


  VertexNormalIndex p(v, normal, vn);
  std::set<VertexNormalIndex>::iterator it=vni.find(p);

  // add the vertex, if a duplicate cannot be found

  if (it != vni.end())
  {
    return it->index;
  }
  else
  {
    vni.insert(p);
    mesh->setVertex(vn, v);
    vn++;

    return p.index;
  }
}

}

Model *loadSTL(const char *name, bool merge_double_vertices)
{
  Mesh *mesh=0;

  // open file

  std::ifstream in;

  in.exceptions(std::ios_base::failbit | std::ios_base::badbit | std::ios_base::eofbit);
  in.open(name, std::ios_base::binary);

  // read header

  gmath::Matrix33d Rc;
  gmath::Vector3d Tc;
  bool has_campose=false;

  {
    char header[81];

    in.read(header, 80);
    header[80]='\0';

    std::string sheader(header);

    if (sheader.compare(0, 8, "campose=") == 0)
    {
      std::istringstream in(sheader.substr(8));

      gmath::Vector6d pose;
      in >> pose;

      if (in.good())
      {
        Rc=gmath::getRotation(pose);
        Tc=gmath::getTranslation(pose);
        has_campose=true;
      }
    }
  }

  // read number of trinagles

  uint32_t n;
  n=stlReadUint32(in);

  try
  {
    // create mesh

    mesh=new Mesh();

    if (has_campose)
    {
      mesh->setDefCameraRT(Rc, Tc);
    }

    mesh->resizeVertexList(3*n, false, false);
    mesh->resizeTriangleList(n);

    std::set<VertexNormalIndex> vni;

    // read all triangles

    int vn=0;

    for (uint32_t i=0; i<n; i++)
    {
      // read triangle normal

      gmath::Vector3f tnormal=stlReadVector3f(in);

      // read vertices and either find a duplicate or add them

      unsigned int t0, t1, t2;

      if (merge_double_vertices)
      {
        t0=findOrStoreVertex(mesh, vni, vn, stlReadVector3f(in), tnormal);
        t1=findOrStoreVertex(mesh, vni, vn, stlReadVector3f(in), tnormal);
        t2=findOrStoreVertex(mesh, vni, vn, stlReadVector3f(in), tnormal);
      }
      else
      {
        t0=vn++;
        t1=vn++;
        t2=vn++;

        mesh->setVertex(t0, stlReadVector3f(in));
        mesh->setVertex(t1, stlReadVector3f(in));
        mesh->setVertex(t2, stlReadVector3f(in));
      }

      // store triangle indices

      mesh->setTriangleIndex(i, t0, t1, t2);

      // skip padding

      in.ignore(1);
      in.ignore(1);
    }

    // reduce list of vertices

    if (vn < static_cast<int>(3*n))
    {
      mesh->resizeVertexList(vn, false, false);
    }

    // calculate per vertex normals

    mesh->recalculateNormals();
  }
  catch (...)
  {
    delete mesh;
    throw;
  }

  in.close();

  return mesh;
}

Model *loadDepth(const char *name, const char *spath, bool verbose)
{
  // load view

  gimage::View view;
  loadView(view, name, spath, verbose);

  // read optional maximum step size parameter

  float dstep=view.getDepthStep();

  std::vector<std::string> list;
  gutil::split(list, name, ',');

  for (size_t i=1; i<list.size(); i++)
  {
    if (list[i].compare(0, 2, "s=") == 0)
    {
      dstep=static_cast<float>(atof(list[i].substr(2).c_str()));
    }
  }

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
  const gimage::ImageU8 *image=&view.getImage();

  if (image->getWidth() > 0 && image->getHeight() > 0)
  {
    ColoredMesh *cmesh=new ColoredMesh();
    cmesh->resizeVertexList(n, true, true);

    // resize image if required

    gimage::ImageU8 dsimage;

    {
      int ds=(image->getWidth()+depth.getWidth()-1)/depth.getWidth();

      if (ds > 1)
      {
        dsimage=gimage::downscaleImage(*image, ds);
        image=&dsimage;
      }
      else if (image->getWidth() < depth.getWidth() || image->getHeight() < depth.getHeight())
      {
        dsimage=gimage::resizeImageBilinear(*image, depth.getWidth(), depth.getHeight());
        image=&dsimage;
      }
    }

    // store colors

    n=0;

    for (long k=0; k<image->getHeight(); k++)
    {
      for (long i=0; i<image->getWidth(); i++)
      {
        if (depth.isValid(i, k))
        {
          if (image->getDepth() == 3)
          {
            cmesh->setColorComp(n, 0, image->get(i, k, 0));
            cmesh->setColorComp(n, 1, image->get(i, k, 1));
            cmesh->setColorComp(n, 2, image->get(i, k, 2));
          }
          else
          {
            cmesh->setColorComp(n, 0, image->get(i, k, 0));
            cmesh->setColorComp(n, 1, image->get(i, k, 0));
            cmesh->setColorComp(n, 2, image->get(i, k, 0));
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

Model *createModelFromContours(const char *name, double scale, double height)
{
  // load image

  gimage::ImageU8 image;
  gimage::getImageIO().load(image, name);

  // extract contours

  std::vector<gimage::Polygon> pl;
  extractContours(pl, image, 0.25f);

  // create mesh from contours

  Mesh *mesh=new Mesh();

  try
  {
    createMeshFromContours(mesh, pl, scale, height);
  }
  catch (...)
  {
    delete mesh;
    throw;
  }

  return mesh;
}

}
