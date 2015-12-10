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

#ifndef GVR_POINTCLOUD_H
#define GVR_POINTCLOUD_H

#include <vector>

#include "model.h"

namespace gvr
{

class PointCloud : public Model
{
  private:
  
    int  n;
    float *vertex;
    float *scanprop; // size, depth error and confidence
    float *scanpos;
    
    PointCloud(const PointCloud &);
    PointCloud& operator=(const PointCloud &);
    
  public:
    
    PointCloud();
    PointCloud(const PointCloud &p, const std::vector<bool> &vused);
    virtual ~PointCloud();
    
    virtual void translate(const gmath::Vector3d &v);
    virtual void addExtend(gmath::Vector3d &emin, gmath::Vector3d &emax) const;
    
    virtual void resizeVertexList(int vn, bool with_scanprop, bool with_scanpos);
    
    int getVertexCount() const { return n; }
    
    float *getVertexArray() { return vertex; };
    float getVertexComp(int i, int k) const { return vertex[3*i+k]; }
    void setVertexComp(int i, int k, float v) { vertex[3*i+k]=v; }
    
    void setVertex(int i, const gmath::SVector<float, 3> &v)
    {
      vertex[3*i]=v[0];
      vertex[3*i+1]=v[1];
      vertex[3*i+2]=v[2];
    }
    
    float *getScanPropArray() { return scanprop; };
    
    bool hasScanProp() const { return scanprop != 0; }
    float getScanSize(int i) const { return scanprop[3*i]; }
    float getScanError(int i) const { return scanprop[3*i+1]; }
    float getScanConf(int i) const { return scanprop[3*i+2]; }
    
    void setScanSize(int i, float v) { scanprop[3*i]=v; }
    void setScanError(int i, float v) { scanprop[3*i+1]=v; }
    void setScanConf(int i, float v) { scanprop[3*i+2]=v; }
    
    float *getScanPosArray() { return scanpos; };
    bool hasScanPos() const { return scanpos != 0; }
    float getScanPosComp(int i, int k) const { return scanpos[3*i+k]; }
    void setScanPosComp(int i, int k, float v) { scanpos[3*i+k]=v; }
    
    virtual void addGLObjects(std::vector<GLObject*> &list);
    
    virtual void loadPLY(PLYReader &ply);
    virtual void savePLY(const char *name, bool all=true, ply_encoding enc=ply_binary) const;
};

}

#endif
