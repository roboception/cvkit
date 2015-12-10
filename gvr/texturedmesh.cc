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

#include "texturedmesh.h"
#include "multitexturedmesh.h"

#ifdef INCLUDE_GL
#include "gltexturedmesh.h"
#endif

#include <algorithm>

#include <gutil/misc.h>
#include <gutil/exception.h>

using std::string;
using std::vector;
using std::ostringstream;
using std::min;

using gutil::trim;

namespace gvr
{

TexturedMesh::TexturedMesh()
{
    uv=0;
}

TexturedMesh::TexturedMesh(const MultiTexturedMesh &p, int t, const vector<bool> &vused,
  const vector<bool> &tused) : Mesh(p, vused, tused)
{
    basepath=p.getBasePath();
    name=p.getTextureName(t);
    
      // compute translation of vertex indices
    
    vector<unsigned int> index;
    index.resize(p.getVertexCount());
    
    unsigned int k=0;
    for (int i=0; i<p.getVertexCount(); i++)
    {
      if (vused[i])
        index[i]=k++;
    }
    
      // convert per triangle corner texture coordinate into per vertex texture
      // coordinates
    
    uv=new float [2*getVertexCount()];

    for (int i=2*getVertexCount()-1; i>=0; i--)
      uv[i]=-1;
    
    int ii=0;
    for (int i=0; i<p.getTriangleCount(); i++)
    {
      if (tused[i])
      {
        if (p.getTextureIndex(i) == t)
        {
          for (int c=0; c<3; c++)
          {
            int k=index[p.getTriangleIndex(i, c)];
            float u=p.getTextureCoordComp(i, c, 0);
            float v=p.getTextureCoordComp(i, c, 1);
            
            if (uv[2*k] == -1)
            {
              uv[2*k]=u;
              uv[2*k+1]=v;
            }
            else if (uv[2*k] != u || uv[2*k+1] != v)
            {
              int vn=getVertexCount();
              resizeVertexList(vn+1, hasScanProp(), hasScanPos());
              
              setVertexComp(vn, 0, getVertexComp(k, 0));
              setVertexComp(vn, 1, getVertexComp(k, 1));
              setVertexComp(vn, 2, getVertexComp(k, 2));
              
              if (hasScanProp())
              {
                setScanSize(vn, getScanSize(k));
                setScanError(vn, getScanError(k));
                setScanConf(vn, getScanConf(k));
              }
              
              if (hasScanPos())
              {
                setScanPosComp(vn, 0, getScanPosComp(k, 0));
                setScanPosComp(vn, 1, getScanPosComp(k, 1));
                setScanPosComp(vn, 2, getScanPosComp(k, 2));
              }
              
              setNormalComp(vn, 0, getNormalComp(k, 0));
              setNormalComp(vn, 1, getNormalComp(k, 1));
              setNormalComp(vn, 2, getNormalComp(k, 2));
              
              uv[2*vn]=u;
              uv[2*vn+1]=v;
              
              setTriangleIndex(ii, c, vn);
            }
          }
        }
        
        ii++;
      }
    }
}

TexturedMesh::~TexturedMesh()
{
    delete [] uv;
}

void TexturedMesh::resizeVertexList(int vn, bool with_scanprop, bool with_scanpos)
{
    float *p=new float [2*vn];
    
    for (int i=2*min(getVertexCount(), vn)-1; i>=0; i--)
      p[i]=uv[i];
    
    for (int i=2*min(getVertexCount(), vn); i<2*vn; i++)
      p[i]=0;
    
    delete [] uv;
    
    uv=p;
    
    Mesh::resizeVertexList(vn, with_scanprop, with_scanpos);
}

void TexturedMesh::addGLObjects(vector<GLObject*> &list)
{
#ifdef INCLUDE_GL
    list.push_back(new GLTexturedMesh(*this));
#else
    assert(false);
#endif
}

void TexturedMesh::loadPLY(PLYReader &ply)
{
    int vn=static_cast<int>(ply.instancesOfElement("vertex"));
    
    setOriginFromPLY(ply);
    
      // get texture name
    
    basepath="";
    
    vector<string> comment;
    ply.getComments(comment);
    
    for (size_t i=0; i<comment.size(); i++)
    {
      size_t k=comment[i].find("TextureFile");
      
      if (k != 0)
        k=comment[i].find("texturefile");
      
      if (k == 0)
      {
        name=comment[i].substr(11);
        trim(name);
        
        basepath=ply.getName();
        
        size_t j=basepath.find_last_of("/\\");
        
        if (j != basepath.npos)
          basepath=basepath.substr(0, j+1);
        else
          basepath="";
        
        break;
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
    
    FloatArrayReceiver tu(getTextureCoordArray(), 2, 0), tv(getTextureCoordArray(), 2, 1);
    
    ply.setReceiver("vertex", "u", &tu);
    ply.setReceiver("vertex", "v", &tv);
    ply.setReceiver("vertex", "texture_u", &tu);
    ply.setReceiver("vertex", "texture_v", &tv);
    ply.setReceiver("vertex", "s", &tu);
    ply.setReceiver("vertex", "t", &tv);
    
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

void TexturedMesh::savePLY(const char *plyname, bool all, ply_encoding enc) const
{
    PLYWriter ply;
    
    ply.open(plyname, enc);
    
      // define data
    
    setOriginToPLY(ply);
    
    ostringstream out;
    out << "TextureFile " << name;
    ply.addComment(out.str());
    
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
    
    ply.addProperty("u", ply_float32);
    ply.addProperty("v", ply_float32);
    
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
      
      ply.write(getTextureCoordComp(i, 0));
      ply.write(getTextureCoordComp(i, 1));
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
