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

#include "multitexturedmesh.h"
#include "texturedmesh.h"

#ifdef INCLUDE_GL
#include "gltexturedmesh.h"
#endif

#include <gutil/misc.h>
#include <gutil/exception.h>

#include <algorithm>

using std::string;
using std::vector;
using std::min;

using gutil::trim;

namespace gvr
{

MultiTexturedMesh::MultiTexturedMesh()
{
    uvt=0;
}

MultiTexturedMesh::~MultiTexturedMesh()
{
    delete [] uvt;
}

void MultiTexturedMesh::resizeTextureList(int n)
{
    name.resize(n);
}

void MultiTexturedMesh::resizeTriangleList(int tn)
{
    UVT *p=new UVT [tn];
    
    for (int i=min(getTriangleCount(), tn)-1; i>=0; i--)
      p[i]=uvt[i];
    
    delete [] uvt;
    
    uvt=p;
    
    Mesh::resizeTriangleList(tn);
}

bool MultiTexturedMesh::getUsedByTexture(int t, vector<bool> &vused, vector<bool> &tused)
{
    vused.resize(getVertexCount());
    tused.resize(getTriangleCount());
    
    for (int i=0; i<getVertexCount(); i++)
      vused[i]=false;
    
    for (int i=0; i<getTriangleCount(); i++)
      tused[i]=false;
    
    bool ret=false;
    for (int i=getTriangleCount()-1; i>=0; i--)
    {
      if (getTextureIndex(i) == t)
      {
        vused[getTriangleIndex(i, 0)]=true;
        vused[getTriangleIndex(i, 1)]=true;
        vused[getTriangleIndex(i, 2)]=true;
        tused[i]=true;
        ret=true;
      }
    }
    
    return ret;
}

void MultiTexturedMesh::addGLObjects(vector<GLObject*> &list)
{
#ifdef INCLUDE_GL
    vector<bool> vused, tused;
    
      // splitting into individual meshs with single texture for each mesh
    
    for (size_t i=0; i<name.size(); i++)
    {
      if (getUsedByTexture(i, vused, tused))
      {
        TexturedMesh p(*this, i, vused, tused);
        list.push_back(new GLTexturedMesh(p));
      }
    }
#else
    assert(false);
#endif
}

void MultiTexturedMesh::loadPLY(PLYReader &ply)
{
    int vn=static_cast<int>(ply.instancesOfElement("vertex"));
    
    setOriginFromPLY(ply);
    
    basepath=ply.getName();
    
    size_t j=basepath.find_last_of("/\\");
    
    if (j != basepath.npos)
      basepath=basepath.substr(0, j+1);
    else
      basepath="";

      // get names of textures
    
    vector<string> comment;
    ply.getComments(comment);
    
    resizeTextureList(0);
    for (size_t i=0; i<comment.size(); i++)
    {
      size_t k=comment[i].find("TextureFile");
      
      if (k != 0)
        comment[i].find("texturefile");
      
      if (k == 0)
      {
        string s=comment[i].substr(11);
        trim(s);
        
        resizeTextureList(getTextureCount()+1);
        setTextureName(getTextureCount()-1, s);
      }
    }
    
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
    
    MultiTextureCoordReceiver tc(*this);
    MultiTextureIndexReceiver ti(*this);
    
    ply.setReceiver("face", "texcoord", &tc);
    ply.setReceiver("face", "texnumber", &ti);
    
      // read data and calculate normals
    
    ply.readData();
    
    if (ply.getTypeOfProperty("vertex", "nx") == ply_none ||
      ply.getTypeOfProperty("vertex", "ny") == ply_none ||
      ply.getTypeOfProperty("vertex", "nz") == ply_none)
      recalculateNormals();
    else
      normalizeNormals();
}

void MultiTexturedMesh::savePLY(const char *plyname, bool all, ply_encoding enc) const
{
    PLYWriter ply;
    
    ply.open(plyname, enc);
    
      // define header
    
    setOriginToPLY(ply);
    
    for (size_t i=0; i<name.size(); i++)
      ply.addComment("TextureFile "+getTextureName(i));
    
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
    ply.addProperty("texcoord", ply_uint8, ply_float32);
    ply.addProperty("texnumber", ply_int32);
    
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
      
      ply.writeListSize(6);
      ply.write(getTextureCoordComp(i, 0, 0));
      ply.write(getTextureCoordComp(i, 0, 1));
      ply.write(getTextureCoordComp(i, 1, 0));
      ply.write(getTextureCoordComp(i, 1, 1));
      ply.write(getTextureCoordComp(i, 2, 0));
      ply.write(getTextureCoordComp(i, 2, 1));
      
      ply.write(getTextureIndex(i));
    }
    
    ply.close();
}

}
