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

#ifndef GVR_MESH_H
#define GVR_MESH_H

#include "pointcloud.h"

namespace gvr
{

class Mesh : public PointCloud
{
  private:
  
    float *normal;
    
    int n;
    unsigned int *triangle;
  
    Mesh(const Mesh &);
    Mesh& operator=(const Mesh &);
    
  public:
    
    Mesh();
    Mesh(const Mesh &p, const std::vector<bool> &vused, const std::vector<bool> &tused);
    virtual ~Mesh();
    
    virtual void resizeVertexList(int vn, bool with_scanprop, bool with_scanpos);
    virtual void resizeTriangleList(int tn);
    
    int getTriangleCount() const { return n; }
    
    float *getNormalArray() { return normal; }
    float getNormalComp(int i, int k) const { return normal[3*i+k]; }
    void setNormalComp(int i, int k, float x) { normal[3*i+k]=x; }
    
    unsigned int *getTriangleArray() { return triangle; }
    unsigned int getTriangleIndex(int i, int c) const { return triangle[3*i+c]; }
    void setTriangleIndex(int i, int c, unsigned int v) { triangle[3*i+c]=v; }
    
    void setTriangleIndex(int i, unsigned int a, unsigned int b, unsigned int c)
    {
      triangle[3*i]=a;
      triangle[3*i+1]=b;
      triangle[3*i+2]=c;
    }
    
    void normalizeNormals();
    void recalculateNormals();
    
    int getUsedVertices(std::vector<bool> &vused);
    
    virtual void addGLObjects(std::vector<GLObject*> &list);
    
    virtual void loadPLY(PLYReader &ply);
    virtual void savePLY(const char *name, bool all=true, ply_encoding enc=ply_binary) const;
};

class TriangleReceiver : public PLYReceiver
{
  private:
    
    Mesh &p;
  
  public:
  
    TriangleReceiver(Mesh &pc) : p(pc) { }
    
    void setValue(int instance, const PLYValue &value)
    {
      p.setTriangleIndex(instance, 0, value.getUnsignedInt(0));
      p.setTriangleIndex(instance, 1, value.getUnsignedInt(1));
      p.setTriangleIndex(instance, 2, value.getUnsignedInt(2));
    }
};

}

#endif
