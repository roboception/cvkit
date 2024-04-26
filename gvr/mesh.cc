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

#include "mesh.h"

#ifdef INCLUDE_GL
#include "glmesh.h"
#endif

#include "gmath/pose.h"
#include "gutil/exception.h"

#include <limits>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <string.h>

#ifdef INCLUDE_GLU
#include <GL/glu.h>
#include <memory>
#endif

namespace gvr
{

Mesh::Mesh()
{
  normal=0;
  n=0;
  triangle=0;
}

Mesh::Mesh(const Mesh &p, const std::vector<bool> &vused, const std::vector<bool> &tused)
  : PointCloud(p, vused)
{
  normal=new float [3*getVertexCount()];

  unsigned int k=0;

  for (int i=0; i<p.getVertexCount(); i++)
  {
    if (vused[i])
    {
      normal[k++]=p.getNormalComp(i, 0);
      normal[k++]=p.getNormalComp(i, 1);
      normal[k++]=p.getNormalComp(i, 2);
    }
  }

  // adapt indices of triangle vertices

  std::vector<unsigned int> index;
  index.resize(p.getVertexCount());

  k=0;

  for (int i=0; i<p.getVertexCount(); i++)
  {
    if (vused[i])
    {
      index[i]=k++;
    }
  }

  n=0;

  for (int i=0; i<p.getTriangleCount(); i++)
    if (tused[i])
    {
      n++;
    }

  triangle=new unsigned int [3*n];

  k=0;

  for (int i=0; i<p.getTriangleCount(); i++)
  {
    if (tused[i])
    {
      triangle[k++]=index[p.getTriangleIndex(i, 0)];
      triangle[k++]=index[p.getTriangleIndex(i, 1)];
      triangle[k++]=index[p.getTriangleIndex(i, 2)];
    }
  }
}

Mesh::~Mesh()
{
  delete [] normal;
  delete [] triangle;
}

void Mesh::resizeVertexList(int vn, bool with_scanprop, bool with_scanpos)
{
  float *p=new float [3*vn];

  for (int i=3*std::min(getVertexCount(), vn)-1; i>=0; i--)
  {
    p[i]=normal[i];
  }

  for (int i=3*std::min(getVertexCount(), vn); i<3*vn; i++)
  {
    p[i]=0;
  }

  delete [] normal;

  normal=p;

  PointCloud::resizeVertexList(vn, with_scanprop, with_scanpos);
}

void Mesh::resizeTriangleList(int tn)
{
  unsigned int *p=new unsigned int [3*tn];

  for (int i=3*std::min(n, tn)-1; i>=0; i--)
  {
    p[i]=triangle[i];
  }

  delete [] triangle;

  triangle=p;
  n=tn;
}

void Mesh::normalizeNormals()
{
  for (int i=3*(getVertexCount()-1); i>=0; i-=3)
  {
    float len=(float) sqrt(normal[i]*normal[i]+
                           normal[i+1]*normal[i+1]+
                           normal[i+2]*normal[i+2]);

    if (len > 0)
    {
      normal[i]/=len;
      normal[i+1]/=len;
      normal[i+2]/=len;
    }
  }
}

void Mesh::recalculateNormals()
{
  for (int i=3*getVertexCount()-1; i>=0; i--)
  {
    normal[i]=0;
  }

  /* compute normals of triangles and sum for each vertex */

  for (int i=0; i<n; i++)
  {
    float x0=getVertexComp(triangle[3*i+1], 0)-getVertexComp(triangle[3*i], 0);
    float y0=getVertexComp(triangle[3*i+1], 1)-getVertexComp(triangle[3*i], 1);
    float z0=getVertexComp(triangle[3*i+1], 2)-getVertexComp(triangle[3*i], 2);

    float x1=getVertexComp(triangle[3*i+2], 0)-getVertexComp(triangle[3*i], 0);
    float y1=getVertexComp(triangle[3*i+2], 1)-getVertexComp(triangle[3*i], 1);
    float z1=getVertexComp(triangle[3*i+2], 2)-getVertexComp(triangle[3*i], 2);

    float nx=y0*z1-z0*y1;
    float ny=z0*x1-x0*z1;
    float nz=x0*y1-y0*x1;

    float len=sqrt(nx*nx+ny*ny+nz*nz);

    if (len > 0)
    {
      nx/=len;
      ny/=len;
      nz/=len;

      for (int k=0; k<3; k++)
      {
        normal[3*triangle[3*i+k]]+=nx;
        normal[3*triangle[3*i+k]+1]+=ny;
        normal[3*triangle[3*i+k]+2]+=nz;
      }
    }
  }

  /* normalize to length 1 */

  normalizeNormals();
}

int Mesh::getUsedVertices(std::vector<bool> &vused)
{
  vused.resize(getVertexCount());

  for (int i=0; i<getVertexCount(); i++)
  {
    vused[i]=false;
  }

  for (int i=3*getTriangleCount()-1; i>=0; i--)
  {
    vused[triangle[i]]=true;
  }

  int count=0;

  for (int i=0; i<getVertexCount(); i++)
    if (vused[i])
    {
      count++;
    }

  return count;
}

void Mesh::addGLObjects(std::vector<GLObject *> &list)
{
#ifdef INCLUDE_GL
  // check if there are unused vertices

  std::vector<bool> vused;
  int n=getUsedVertices(vused);

  // split into point cloud and mesh

  if (getVertexCount() == n)
  {
    list.push_back(new GLMesh(*this));
  }
  else
  {
    if (n > 0)
    {
      std::vector<bool> tused(getTriangleCount(), true);
      Mesh m(*this, vused, tused);
      list.push_back(new GLMesh(m));
    }

    if (getVertexCount()-n > 0)
    {
      for (int i=0; i<getVertexCount(); i++)
      {
        vused[i]=!vused[i];
      }

      PointCloud p(*this, vused);
      list.push_back(new GLPointCloud(p));
    }
  }

#else
  assert(false);
#endif
}

void Mesh::loadPLY(PLYReader &ply)
{
  int vn=static_cast<int>(ply.instancesOfElement("vertex"));

  setOriginFromPLY(ply);

  // set receiver for vertices

  resizeVertexList(vn,
                   ply.getTypeOfProperty("vertex", "scan_size") != ply_none ||
                   ply.getTypeOfProperty("vertex", "scan_error") != ply_none ||
                   ply.getTypeOfProperty("vertex", "scan_conf") != ply_none,
                   ply.getTypeOfProperty("vertex", "sx") != ply_none &&
                   ply.getTypeOfProperty("vertex", "sy") != ply_none &&
                   ply.getTypeOfProperty("vertex", "sz") != ply_none);

  FloatArrayReceiver vx(getVertexArray(), 3, 0), vy(getVertexArray(), 3, 1), vz(getVertexArray(), 3,
      2);

  ply.setReceiver("vertex", "x", &vx);
  ply.setReceiver("vertex", "y", &vy);
  ply.setReceiver("vertex", "z", &vz);

  FloatArrayReceiver ss(getScanPropArray(), 3, 0), se(getScanPropArray(), 3, 1),
                     sc(getScanPropArray(), 3, 2);

  if (hasScanProp())
  {
    ply.setReceiver("vertex", "scan_size", &ss);
    ply.setReceiver("vertex", "scan_error", &se);
    ply.setReceiver("vertex", "scan_conf", &sc);
  }

  FloatArrayReceiver sx(getScanPosArray(), 3, 0), sy(getScanPosArray(), 3, 1), sz(getScanPosArray(),
      3, 2);

  if (hasScanPos())
  {
    ply.setReceiver("vertex", "sx", &sx);
    ply.setReceiver("vertex", "sy", &sy);
    ply.setReceiver("vertex", "sz", &sz);
  }

  FloatArrayReceiver nx(getNormalArray(), 3, 0), ny(getNormalArray(), 3, 1), nz(getNormalArray(), 3,
      2);

  ply.setReceiver("vertex", "nx", &nx);
  ply.setReceiver("vertex", "ny", &ny);
  ply.setReceiver("vertex", "nz", &nz);

  // set receiver for triangles

  int tn=static_cast<int>(ply.instancesOfElement("face"));

  resizeTriangleList(tn);

  TriangleReceiver tr(*this);

  ply.setReceiver("face", "vertex_index", &tr);
  ply.setReceiver("face", "vertex_indices", &tr);

  // read data and calculate normals

  ply.readData();

  if (ply.getTypeOfProperty("vertex", "nx") == ply_none ||
      ply.getTypeOfProperty("vertex", "ny") == ply_none ||
      ply.getTypeOfProperty("vertex", "nz") == ply_none)
  {
    recalculateNormals();
  }
  else
  {
    normalizeNormals();
  }
}

void Mesh::savePLY(const char *name, bool all, ply_encoding enc) const
{
  PLYWriter ply;

  ply.open(name, enc);

  // define header

  setOriginToPLY(ply);

  ply.addElement("vertex", getVertexCount());

  ply.addProperty("x", ply_float32);
  ply.addProperty("y", ply_float32);
  ply.addProperty("z", ply_float32);

  if (hasScanProp() != 0)
  {
    ply.addProperty("scan_size", ply_float32);

    if (all)
    {
      ply.addProperty("scan_error", ply_float32);
      ply.addProperty("scan_conf", ply_float32);
    }
  }

  if (all && hasScanPos() != 0)
  {
    ply.addProperty("sx", ply_float32);
    ply.addProperty("sy", ply_float32);
    ply.addProperty("sz", ply_float32);
  }

  if (all)
  {
    ply.addProperty("nx", ply_float32);
    ply.addProperty("ny", ply_float32);
    ply.addProperty("nz", ply_float32);
  }

  ply.addElement("face", getTriangleCount());
  ply.addProperty("vertex_indices", ply_uint8, ply_uint32);

  // write data

  for (int i=0; i<getVertexCount(); i++)
  {
    ply.write(getVertexComp(i, 0));
    ply.write(getVertexComp(i, 1));
    ply.write(getVertexComp(i, 2));

    if (hasScanProp() != 0)
    {
      ply.write(getScanSize(i));

      if (all)
      {
        ply.write(getScanError(i));
        ply.write(getScanConf(i));
      }
    }

    if (all && hasScanPos() != 0)
    {
      ply.write(getScanPosComp(i, 0));
      ply.write(getScanPosComp(i, 1));
      ply.write(getScanPosComp(i, 2));
    }

    if (all)
    {
      ply.write(getNormalComp(i, 0));
      ply.write(getNormalComp(i, 1));
      ply.write(getNormalComp(i, 2));
    }
  }

  for (int i=0; i<getTriangleCount(); i++)
  {
    ply.writeListSize(3);
    ply.write(getTriangleIndex(i, 0));
    ply.write(getTriangleIndex(i, 1));
    ply.write(getTriangleIndex(i, 2));
  }

  ply.close();
}

namespace
{

inline void stlWriteUint16(std::ofstream &out, uint16_t v)
{
  // write as little endian
  out.put(static_cast<char>(v&0xff));
  out.put(static_cast<char>((v>>8)&0xff));
}

inline void stlWriteUint32(std::ofstream &out, uint32_t v)
{
  // write as little endian
  out.put(static_cast<char>(v&0xff));
  out.put(static_cast<char>((v>>8)&0xff));
  out.put(static_cast<char>((v>>16)&0xff));
  out.put(static_cast<char>((v>>24)&0xff));
}

inline void stlWriteFloat32(std::ofstream &out, float v)
{
  // reinterpret and uint32 and write as little endian
  uint32_t vi;
  memcpy(reinterpret_cast<char *>(&vi), reinterpret_cast<char *>(&v), 4);
  stlWriteUint32(out, vi);
}

inline void stlWriteVector3f(std::ofstream &out, const gmath::Vector3f &v)
{
  stlWriteFloat32(out, v[0]);
  stlWriteFloat32(out, v[1]);
  stlWriteFloat32(out, v[2]);
}

}

void Mesh::saveSTL(const char *name) const
{
  std::ofstream out;

  out.exceptions(std::ios_base::failbit | std::ios_base::badbit);
  out.open(name, std::ios_base::binary);

  // write header with camera definition

  std::ostringstream header;

  header << "campose=";
  gmath::Vector6d pose=gmath::getPose(getDefCameraR(), getDefCameraT());
  header << std::setprecision(5) << pose;

  while (header.tellp() < 80) header << ' ';

  out.write(header.str().c_str(), 80);

  // write number of triangles

  stlWriteUint32(out, static_cast<uint32_t>(n));

  unsigned int *tp=triangle;
  for (int i=0; i<n; i++)
  {
    // get vertices

    gmath::Vector3f v0=getVertex(tp[0]);
    gmath::Vector3f v1=getVertex(tp[1]);
    gmath::Vector3f v2=getVertex(tp[2]);

    // compute normal of triangle

    gmath::Vector3f vn=gmath::cross(v1-v0, v2-v0);
    float len=gmath::norm(vn);

    if (len > 0)
    {
      vn/=len;
    }

    // store normal

    stlWriteVector3f(out, vn);

    // store vertices

    stlWriteVector3f(out, v0);
    stlWriteVector3f(out, v1);
    stlWriteVector3f(out, v2);

    // store padding

    stlWriteUint16(out, 0);

    tp+=3;
  }

  out.close();
}

#ifdef INCLUDE_GLU

namespace
{

struct VertexData
{
  GLdouble xyz[3];
  int      index;  // vertex index
};

struct TriangleData
{
  int index[3]; // indices of triangle vertices

  TriangleData()
  {
    index[0]=-1;
    index[1]=-1;
    index[2]=-1;
  }

  TriangleData(int a, int b, int c)
  {
    index[0]=a;
    index[1]=b;
    index[2]=c;
  }
};

struct PolygonData
{
  // type of triangle order in tessalation and indices that are used depending
  // on that type

  GLenum type;
  int tindex0;
  int tindex1;
  bool inverse;

  // vertices and triangles with indices to vertices

  int nvertex;
  std::vector<std::shared_ptr<VertexData> > vertex;
  std::vector<TriangleData> triangle;

  PolygonData()
  {
    type=GL_TRIANGLES;
    tindex0=-1;
    tindex1=-1;
    inverse=false;
    nvertex=0;
  }
};

void beginCb(GLenum type, void *user_data)
{
  PolygonData *pd=reinterpret_cast<PolygonData *>(user_data);

  // tesselation starts

  pd->type=type;
  pd->tindex0=-1;
  pd->tindex1=-1;
  pd->inverse=false;
}

void combineCb(GLdouble coords[3], void *[4], GLfloat [4], void **out_data, void *user_data)
{
  PolygonData *pd=reinterpret_cast<PolygonData *>(user_data);

  // tesselation needs to create a new point

  std::shared_ptr<VertexData> vd=std::make_shared<VertexData>();

  vd->index=pd->nvertex++;
  vd->xyz[0]=coords[0];
  vd->xyz[1]=coords[1];
  vd->xyz[2]=coords[2];

  pd->vertex.push_back(vd);

  *out_data=vd.get();
}

void vertexCb(void *vertex_data, void *user_data)
{
  PolygonData *pd=reinterpret_cast<PolygonData *>(user_data);
  VertexData *vd=reinterpret_cast<VertexData *>(vertex_data);

  // create triangles from vertex according to type

  switch (pd->type)
  {
    case GL_TRIANGLES: // Triangles: {0 1 2} {3 4 5} ...
      if (pd->tindex0 < 0)
      {
        pd->tindex0=vd->index;
      }
      else if (pd->tindex1 < 0)
      {
        pd->tindex1=vd->index;
      }
      else
      {
        pd->triangle.push_back(TriangleData(pd->tindex0, pd->tindex1, vd->index));
        pd->tindex0=-1;
        pd->tindex1=-1;
      }
      break;

    case GL_TRIANGLE_STRIP: // Triangles: {0 1 2} {2 1 3} {2 3 4} {4 3 5} ...
      if (pd->tindex0 < 0)
      {
        pd->tindex0=vd->index;
      }
      else if (pd->tindex1 < 0)
      {
        pd->tindex1=vd->index;
      }
      else
      {
        if (pd->inverse)
        {
          pd->triangle.push_back(TriangleData(pd->tindex1, pd->tindex0, vd->index));
        }
        else
        {
          pd->triangle.push_back(TriangleData(pd->tindex0, pd->tindex1, vd->index));
        }

        pd->tindex0=pd->tindex1;
        pd->tindex1=vd->index;
        pd->inverse=!pd->inverse;
      }
      break;

    case GL_TRIANGLE_FAN: // Triangle: {0 1 2} {0 2 3} {0 3 4} {0 4 5} ...
      if (pd->tindex0 < 0)
      {
        pd->tindex0=vd->index;
      }
      else if (pd->tindex1 < 0)
      {
        pd->tindex1=vd->index;
      }
      else
      {
        pd->triangle.push_back(TriangleData(pd->tindex0, pd->tindex1, vd->index));
        pd->tindex1=vd->index;
      }
      break;

    default:
      std::cerr << "Internal error: Unknown type" << std::endl;
      break;
  }
}

void endCb(void *)
{
  // nothing to be done here
}

}

void TriangleReceiver::setPolygon(int instance, const PLYValue &value)
{
  PolygonData pd;

  pd.nvertex=p.getVertexCount();

  // get vertex coordinates of polygon

  for (int i=0; i<value.getListSize(); i++)
  {
    std::shared_ptr<VertexData> vd=std::make_shared<VertexData>();

    vd->index=static_cast<int>(value.getUnsignedInt(i));
    vd->xyz[0]=p.getVertexComp(vd->index, 0);
    vd->xyz[1]=p.getVertexComp(vd->index, 1);
    vd->xyz[2]=p.getVertexComp(vd->index, 2);

    pd.vertex.push_back(vd);
  }

  // triangulate outer contour with inner contours as holes for top face
  // using tesselation function from OpenGL utility library libGLU

  {
    GLUtesselator* tobj=gluNewTess();
#ifdef WIN32
    gluTessCallback(tobj, GLU_TESS_BEGIN_DATA, reinterpret_cast<void (__stdcall *) ()>(&beginCb));
    gluTessCallback(tobj, GLU_TESS_COMBINE_DATA, reinterpret_cast<void (__stdcall *) ()>(&combineCb));
    gluTessCallback(tobj, GLU_TESS_VERTEX_DATA, reinterpret_cast<void (__stdcall *) ()>(&vertexCb));
    gluTessCallback(tobj, GLU_TESS_END_DATA, reinterpret_cast<void (__stdcall *) ()>(&endCb));
#else
    gluTessCallback(tobj, GLU_TESS_BEGIN_DATA, reinterpret_cast<GLvoid (*) ()>(&beginCb));
    gluTessCallback(tobj, GLU_TESS_COMBINE_DATA, reinterpret_cast<GLvoid (*) ()>(&combineCb));
    gluTessCallback(tobj, GLU_TESS_VERTEX_DATA, reinterpret_cast<GLvoid (*) ()>(&vertexCb));
    gluTessCallback(tobj, GLU_TESS_END_DATA, reinterpret_cast<GLvoid (*) ()>(&endCb));
#endif

    // outer contours counterclockwise, inner contours clockwise
    gluTessProperty(tobj, GLU_TESS_WINDING_RULE, GLU_TESS_WINDING_NONZERO);
    gluTessProperty(tobj, GLU_TESS_BOUNDARY_ONLY, GL_FALSE);
    gluTessProperty(tobj, GLU_TESS_TOLERANCE, 0);

    gluTessBeginPolygon(tobj, &pd);

    gluTessBeginContour(tobj);

    for (size_t k=0; k<pd.vertex.size(); k++)
    {
      gluTessVertex(tobj, pd.vertex[k]->xyz, pd.vertex[k].get());
    }

    gluTessEndContour(tobj);

    gluTessEndPolygon(tobj);
    gluDeleteTess(tobj);
  }

  // add vertices, if created during tesselation

  if (pd.nvertex > p.getVertexCount())
  {
    int i=p.getVertexCount();
    int k=static_cast<int>(pd.vertex.size())-(pd.nvertex-p.getVertexCount());

    p.resizeVertexList(pd.nvertex, p.hasScanProp(), p.hasScanPos());

    while (i < p.getVertexCount())
    {
      p.setVertex(i++, static_cast<float>(pd.vertex[k]->xyz[0]),
        static_cast<float>(pd.vertex[k]->xyz[1]), static_cast<float>(pd.vertex[k]->xyz[2]));
    }
  }

  // set first triangle as instance with given number

  if (pd.triangle.size() > 0)
  {
    p.setTriangleIndex(instance, pd.triangle[0].index[0], pd.triangle[0].index[1],
      pd.triangle[0].index[2]);

    // add all other triangles

    int k=p.getTriangleCount();
    p.resizeTriangleList(p.getTriangleCount()+static_cast<int>(pd.triangle.size())-1);
    for (size_t i=1; i<pd.triangle.size(); i++)
    {
      p.setTriangleIndex(k++, pd.triangle[i].index[0], pd.triangle[i].index[1],
        pd.triangle[i].index[2]);
    }
  }
  else
  {
    std::cerr << "Warning: Cannot triangulate polygon with " << pd.vertex.size() << " vertices" << std::endl;
  }
}

#else

void TriangleReceiver::setPolygon(int instance, const PLYValue &value)
{
  p.setTriangleIndex(instance, value.getUnsignedInt(0), value.getUnsignedInt(1),
    value.getUnsignedInt(2));

  std::cerr << "Warning: Cannot triangulate polygons in PLY file. Please recompile with GLU support." << std::endl;
}

#endif

}
