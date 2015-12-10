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

#ifndef GVR_MULTITEXTUREDMESH_H
#define GVR_MULTITEXTUREDMESH_H

#include "mesh.h"

#include <string>
#include <vector>

namespace gvr
{

/**
  This class implements a mesh with multiple texture images with texture
  coordinates for each corner of each triangle. See TexturedMesh for 
  a mesh with texture coordinates for each vertex.
*/

class MultiTexturedMesh : public Mesh
{
  private:
    
    std::string basepath;
    std::vector<std::string> name;
    
    struct UVT
    {
      float uv[6];
      int   t;
    } *uvt;
    
    MultiTexturedMesh(const MultiTexturedMesh &);
    MultiTexturedMesh& operator=(const MultiTexturedMesh &);
    
  public:
    
    MultiTexturedMesh();
    virtual ~MultiTexturedMesh();
    
    virtual void resizeTextureList(int n);
    virtual void resizeTriangleList(int tn);
    
    const std::string &getBasePath() const { return basepath; }
    void setBasePath(std::string s) { basepath=s; }
    
    int getTextureCount() const { return name.size(); }
    const std::string &getTextureName(int i) const { return name[i]; }
    void setTextureName(int i, std::string s) { name[i]=s; }
    
      // components k (i.e. 0 or 1) of texture coordinates for corner c (i.e.
      // 0, 1 or 2) of each triangle i
    
    float getTextureCoordComp(int i, int c, int k) const { return uvt[i].uv[2*c+k]; }
    void setTextureCoordComp(int i, int c, int k, float v) { uvt[i].uv[2*c+k]=v; }
    
      // index of texture that belongs to triangle i
    
    int getTextureIndex(int i) const { return uvt[i].t; }
    void setTextureIndex(int i, int t) const { uvt[i].t=t; }
    
    bool getUsedByTexture(int t, std::vector<bool> &vused, std::vector<bool> &tused);
    virtual void addGLObjects(std::vector<GLObject*> &list);
    
    virtual void loadPLY(PLYReader &ply);
    virtual void savePLY(const char *name, bool all=true, ply_encoding enc=ply_binary) const;
};

class MultiTextureCoordReceiver : public PLYReceiver
{
  private:
    
    MultiTexturedMesh &p;
  
  public:
  
    MultiTextureCoordReceiver(MultiTexturedMesh &pc) : p(pc) { }
    
    void setValue(int instance, const PLYValue &value)
    {
      p.setTextureCoordComp(instance, 0, 0, value.getFloat(0));
      p.setTextureCoordComp(instance, 0, 1, value.getFloat(1));
      p.setTextureCoordComp(instance, 1, 0, value.getFloat(2));
      p.setTextureCoordComp(instance, 1, 1, value.getFloat(3));
      p.setTextureCoordComp(instance, 2, 0, value.getFloat(4));
      p.setTextureCoordComp(instance, 2, 1, value.getFloat(5));
    }
};

class MultiTextureIndexReceiver : public PLYReceiver
{
  private:
    
    MultiTexturedMesh &p;
  
  public:
  
    MultiTextureIndexReceiver(MultiTexturedMesh &pc) : p(pc) { }
    
    void setValue(int instance, const PLYValue &value)
    {
      p.setTextureIndex(instance, value.getInt());
    }
};
}

#endif
