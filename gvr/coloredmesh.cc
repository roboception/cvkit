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

#include "coloredmesh.h"
#include "coloredpointcloud.h"

#ifdef INCLUDE_GL
#include "glcoloredpointcloud.h"
#include "glcoloredmesh.h"
#endif

#include <gutil/exception.h>

#include <algorithm>

using std::vector;
using std::min;

namespace gvr
{

ColoredMesh::ColoredMesh()
{
    rgb=0;
}

ColoredMesh::ColoredMesh(const ColoredMesh &p, const vector<bool> &vused,
  const vector<bool> &tused) : Mesh(p, vused, tused)
{
    rgb=new unsigned char [3*getVertexCount()];
    
    int k=0;
    for (int i=0; i<p.getVertexCount(); i++)
    {
      if (vused[i])
      {
        rgb[k++]=p.getColorComp(i, 0);
        rgb[k++]=p.getColorComp(i, 1);
        rgb[k++]=p.getColorComp(i, 2);
      }
    }
}

ColoredMesh::~ColoredMesh()
{
    delete [] rgb;
}

void ColoredMesh::resizeVertexList(int vn, bool with_scanprop, bool with_scanpos)
{
    unsigned char *p=new unsigned char [3*vn];
    
    for (int i=3*min(getVertexCount(), vn)-1; i>=0; i--)
      p[i]=rgb[i];
    
    for (int i=3*min(getVertexCount(), vn); i<3*vn; i++)
      p[i]=0;
    
    delete [] rgb;
    
    rgb=p;
    
    Mesh::resizeVertexList(vn, with_scanprop, with_scanpos);
}

void ColoredMesh::addGLObjects(vector<GLObject*> &list)
{
#ifdef INCLUDE_GL
      // check if there are unused vertices
    
    vector<bool> vused;
    int n=getUsedVertices(vused);
    
      // split into point cloud and mesh
    
    if (getVertexCount() == n)
    {
      list.push_back(new GLColoredMesh(*this));
    }
    else
    {
      if (n > 0)
      {
        vector<bool> tused(getTriangleCount(), true);
        ColoredMesh m(*this, vused, tused);
        list.push_back(new GLColoredMesh(m));
      }
      
      if (getVertexCount()-n > 0)
      {
        for (int i=0; i<getVertexCount(); i++)
          vused[i]=!vused[i];
        
        ColoredPointCloud p(*this, vused);
        list.push_back(new GLColoredPointCloud(p));
      }
    }
#else
    assert(false);
#endif
}

void ColoredMesh::loadPLY(PLYReader &ply)
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
    
    float scale=1.0f;
    ply_type type=ply.getTypeOfProperty("vertex", "diffuse_red");
    
    if (type == ply_none)
      type=ply.getTypeOfProperty("vertex", "red");
    
    if (type == ply_uint16)
      scale=1.0f/256;
    
    if (type == ply_float32 || type == ply_float64)
      scale=255;
    
    UInt8Receiver cr(getColorArray(), 0, scale), cg(getColorArray(), 1, scale), cb(getColorArray(), 2, scale);
    
    ply.setReceiver("vertex", "diffuse_red", &cr);
    ply.setReceiver("vertex", "diffuse_green", &cg);
    ply.setReceiver("vertex", "diffuse_blue", &cb);
    
    ply.setReceiver("vertex", "red", &cr);
    ply.setReceiver("vertex", "green", &cg);
    ply.setReceiver("vertex", "blue", &cb);
    
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

void ColoredMesh::savePLY(const char *name, bool all, ply_encoding enc) const
{
    PLYWriter ply;
    
    ply.open(name, enc);
    
      // define data
    
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
    
    ply.addProperty("diffuse_red", ply_uint8);
    ply.addProperty("diffuse_green", ply_uint8);
    ply.addProperty("diffuse_blue", ply_uint8);
    
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
      
      ply.write(getColorComp(i, 0));
      ply.write(getColorComp(i, 1));
      ply.write(getColorComp(i, 2));
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
