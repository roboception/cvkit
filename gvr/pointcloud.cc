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

#include "pointcloud.h"

#ifdef INCLUDE_GL
#include "glpointcloud.h"
#endif

#include <gutil/exception.h>

#include <limits>
#include <algorithm>

using std::vector;
using std::numeric_limits;
using std::min;
using std::max;

using gmath::Vector3d;

namespace gvr
{

PointCloud::PointCloud()
{
    n=0;
    vertex=0;
    scanprop=0;
    scanpos=0;
}

PointCloud::PointCloud(const PointCloud &p, const vector<bool> &vused) : Model(p)
{
    n=0;
    vertex=0;
    scanprop=0;
    scanpos=0;
    
    for (int i=vused.size()-1; i>=0; i--)
      if (vused[i])
        n++;
    
    vertex=new float [3*n];
    
    int k=0;
    for (int i=0; i<p.getVertexCount(); i++)
    {
      if (vused[i])
      {
        vertex[k++]=p.getVertexComp(i, 0);
        vertex[k++]=p.getVertexComp(i, 1);
        vertex[k++]=p.getVertexComp(i, 2);
      }
    }
    
    if (p.hasScanProp())
    {
      scanprop=new float [3*n];
      
      k=0;
      for (int i=0; i<p.getVertexCount(); i++)
      {
        if (vused[i])
        {
          scanprop[k++]=p.getScanSize(i);
          scanprop[k++]=p.getScanError(i);
          scanprop[k++]=p.getScanConf(i);
        }
      }
    }
    
    if (p.hasScanPos())
    {
      scanpos=new float [3*n];
      
      k=0;
      for (int i=0; i<p.getVertexCount(); i++)
      {
        if (vused[i])
        {
          scanpos[k++]=p.getScanPosComp(i, 0);
          scanpos[k++]=p.getScanPosComp(i, 1);
          scanpos[k++]=p.getScanPosComp(i, 2);
        }
      }
    }
}

PointCloud::~PointCloud()
{
    delete [] vertex;
    delete [] scanprop;
    delete [] scanpos;
}

void PointCloud::translate(const Vector3d &v)
{
    Vector3d t=v+getOrigin();
    
    if (t[0] != 0 || t[1] != 0 || t[2] != 0)
    {
      for (int i=3*(n-1); i>=0; i-=3)
      {
        vertex[i]+=t[0];
        vertex[i+1]+=t[1];
        vertex[i+2]+=t[2];
      }
      
      if (scanpos != 0)
      {
        for (int i=3*(n-1); i>=0; i-=3)
        {
          scanpos[i]+=t[0];
          scanpos[i+1]+=t[1];
          scanpos[i+2]+=t[2];
        }
      }
    }
    
    setOrigin(Vector3d(0, 0, 0));
    setDefCameraRT(getDefCameraR(), getDefCameraT()+t);
}

void PointCloud::addExtend(Vector3d &emin, Vector3d &emax) const
{
    float xmin=numeric_limits<float>::max();
    float ymin=numeric_limits<float>::max();
    float zmin=numeric_limits<float>::max();
    float xmax=-numeric_limits<float>::max();
    float ymax=-numeric_limits<float>::max();
    float zmax=-numeric_limits<float>::max();
    
    for (int i=3*(n-1); i>=0; i-=3)
    {
      xmin=min(xmin, vertex[i]);
      ymin=min(ymin, vertex[i+1]);
      zmin=min(zmin, vertex[i+2]);
      
      xmax=max(xmax, vertex[i]);
      ymax=max(ymax, vertex[i+1]);
      zmax=max(zmax, vertex[i+2]);
    }
    
    emin[0]=min(emin[0], getOrigin()[0]+xmin);
    emin[1]=min(emin[1], getOrigin()[1]+ymin);
    emin[2]=min(emin[2], getOrigin()[2]+zmin);
    
    emax[0]=max(emax[0], getOrigin()[0]+xmax);
    emax[1]=max(emax[1], getOrigin()[1]+ymax);
    emax[2]=max(emax[2], getOrigin()[2]+zmax);
}

void PointCloud::resizeVertexList(int vn, bool with_scanprop, bool with_scanpos)
{
    float *p=new float [3*vn];
    
    for (int i=3*min(n, vn)-1; i>=0; i--)
      p[i]=vertex[i];
    
    for (int i=3*min(n, vn); i<3*vn; i++)
      p[i]=0;
    
    delete [] vertex;
    
    vertex=p;
    p=0;
    
    if (with_scanprop)
    {
      p=new float [3*vn];
      
      for (int i=3*min(n, vn)-1; i>=0; i--)
        p[i]=scanprop[i];
      
      for (int i=min(n, vn); i<vn; i++)
      {
        p[3*i]=0;
        p[3*i+1]=0;
        p[3*i+2]=1;
      }
    }
    
    delete [] scanprop;
    
    scanprop=p;
    p=0;
    
    if (with_scanpos)
    {
      p=new float [3*vn];
      
      for (int i=3*min(n, vn)-1; i>=0; i--)
        p[i]=scanpos[i];
      
      for (int i=3*min(n, vn); i<3*vn; i++)
        p[i]=0;
    }
    
    delete [] scanpos;
    
    scanpos=p;
    
    n=vn;
}

void PointCloud::addGLObjects(vector<GLObject*> &list)
{
#ifdef INCLUDE_GL
    list.push_back(new GLPointCloud(*this));
#else
    assert(false);
#endif
}

void PointCloud::loadPLY(PLYReader &ply)
{
    int vn=static_cast<int>(ply.instancesOfElement("vertex"));
    
    setOriginFromPLY(ply);
    
      // set receiver for point cloud
    
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
    
      // read data
    
    ply.readData();
}

void PointCloud::savePLY(const char *name, bool all, ply_encoding enc) const
{
    PLYWriter ply;
    
      // define header
    
    ply.open(name, enc);
    
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
    }
    
    ply.close();
}

}
