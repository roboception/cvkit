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

#include <gutil/exception.h>

#include <limits>
#include <algorithm>

using std::vector;
using std::min;

namespace gvr
{

Mesh::Mesh()
{
    normal=0;
    n=0;
    triangle=0;
}

Mesh::Mesh(const Mesh &p, const vector<bool> &vused, const vector<bool> &tused)
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
    
    vector<unsigned int> index;
    index.resize(p.getVertexCount());
    
    k=0;
    for (int i=0; i<p.getVertexCount(); i++)
    {
      if (vused[i])
        index[i]=k++;
    }
    
    n=0;
    for (int i=0; i<p.getTriangleCount(); i++)
      if (tused[i])
        n++;
    
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
    
    for (int i=3*min(getVertexCount(), vn)-1; i>=0; i--)
      p[i]=normal[i];
    
    for (int i=3*min(getVertexCount(), vn); i<3*vn; i++)
      p[i]=0;
    
    delete [] normal;
    
    normal=p;
    
    PointCloud::resizeVertexList(vn, with_scanprop, with_scanpos);
}

void Mesh::resizeTriangleList(int tn)
{
    unsigned int *p=new unsigned int [3*tn];
    
    for (int i=3*min(n, tn)-1; i>=0; i--)
      p[i]=triangle[i];
    
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
      normal[i]=0;
    
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

int Mesh::getUsedVertices(vector<bool> &vused)
{
    vused.resize(getVertexCount());
    
    for (int i=0; i<getVertexCount(); i++)
      vused[i]=false;
    
    for (int i=3*getTriangleCount()-1; i>=0; i--)
      vused[triangle[i]]=true;
    
    int count=0;
    for (int i=0; i<getVertexCount(); i++)
      if (vused[i])
        count++;
    
    return count;
}

void Mesh::addGLObjects(vector<GLObject*> &list)
{
#ifdef INCLUDE_GL
      // check if there are unused vertices
    
    vector<bool> vused;
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
        vector<bool> tused(getTriangleCount(), true);
        Mesh m(*this, vused, tused);
        list.push_back(new GLMesh(m));
      }
      
      if (getVertexCount()-n > 0)
      {
        for (int i=0; i<getVertexCount(); i++)
          vused[i]=!vused[i];
        
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
    
    FloatArrayReceiver vx(getVertexArray(), 3, 0), vy(getVertexArray(), 3, 1), vz(getVertexArray(), 3, 2);
    
    ply.setReceiver("vertex", "x", &vx);
    ply.setReceiver("vertex", "y", &vy);
    ply.setReceiver("vertex", "z", &vz);
    
    FloatArrayReceiver ss(getScanPropArray(), 3, 0), se(getScanPropArray(), 3, 1), sc(getScanPropArray(), 3, 2);
    if (hasScanProp())
    {
      ply.setReceiver("vertex", "scan_size", &ss);
      ply.setReceiver("vertex", "scan_error", &se);
      ply.setReceiver("vertex", "scan_conf", &sc);
    }
    
    FloatArrayReceiver sx(getScanPosArray(), 3, 0), sy(getScanPosArray(), 3, 1), sz(getScanPosArray(), 3, 2);
    
    if (hasScanPos())
    {
      ply.setReceiver("vertex", "sx", &sx);
      ply.setReceiver("vertex", "sy", &sy);
      ply.setReceiver("vertex", "sz", &sz);
    }
    
    FloatArrayReceiver nx(getNormalArray(), 3, 0), ny(getNormalArray(), 3, 1), nz(getNormalArray(), 3, 2);
    
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
      recalculateNormals();
    else
      normalizeNormals();
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

}
