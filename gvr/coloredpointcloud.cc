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

#include "coloredpointcloud.h"
#include "coloredmesh.h"

#ifdef INCLUDE_GL
#include "glcoloredpointcloud.h"
#endif

#include <gutil/exception.h>

#include <algorithm>

using std::vector;
using std::min;

namespace gvr
{

ColoredPointCloud::ColoredPointCloud()
{
    rgb=0;
}

ColoredPointCloud::ColoredPointCloud(const ColoredMesh &p, const vector<bool> &vused) : PointCloud(p, vused)
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

ColoredPointCloud::~ColoredPointCloud()
{
    delete [] rgb;
}

void ColoredPointCloud::resizeVertexList(int vn, bool with_scanprop, bool with_scanpos)
{
    unsigned char *p=new unsigned char [3*vn];
    
    for (int i=3*min(getVertexCount(), vn)-1; i>=0; i--)
      p[i]=rgb[i];
    
    for (int i=3*min(getVertexCount(), vn); i<3*vn; i++)
      p[i]=0;
    
    delete [] rgb;
    
    rgb=p;
    
    PointCloud::resizeVertexList(vn, with_scanprop, with_scanpos);
}

void ColoredPointCloud::addGLObjects(vector<GLObject*> &list)
{
#ifdef INCLUDE_GL
    list.push_back(new GLColoredPointCloud(*this));
#else
    assert(false);
#endif
}

void ColoredPointCloud::loadPLY(PLYReader &ply)
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
    
      // set receiver for color
    
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
    
      // read data
    
    ply.readData();
}

void ColoredPointCloud::savePLY(const char *name, bool all, ply_encoding enc) const
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
    
    ply.addProperty("diffuse_red", ply_uint8);
    ply.addProperty("diffuse_green", ply_uint8);
    ply.addProperty("diffuse_blue", ply_uint8);
    
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
      
      ply.write(getColorComp(i, 0));
      ply.write(getColorComp(i, 1));
      ply.write(getColorComp(i, 2));
    }
    
    ply.close();
}

}
