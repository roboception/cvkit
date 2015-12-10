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

#ifndef GVR_COLOREDMESH_H
#define GVR_COLOREDMESH_H

#include "mesh.h"

namespace gvr
{

class ColoredMesh : public Mesh
{
  private:
  
    unsigned char *rgb;
  
    ColoredMesh(const ColoredMesh &);
    ColoredMesh& operator=(const ColoredMesh &);
    
  public:
    
    ColoredMesh();
    ColoredMesh(const ColoredMesh &p, const std::vector<bool> &vused, const std::vector<bool> &tused);
    virtual ~ColoredMesh();
    
    virtual void resizeVertexList(int vn, bool with_scanprop, bool with_scanpos);
    
    unsigned char *getColorArray() { return rgb; }
    unsigned char getColorComp(int i, int k) const { return rgb[3*i+k]; }
    void setColorComp(int i, int k, unsigned char c) { rgb[3*i+k]=c; }
    
    void setColor(int i, unsigned char r, unsigned char g, unsigned char b)
    {
      rgb[3*i]=r;
      rgb[3*i+1]=g;
      rgb[3*i+2]=b;
    }
    
    virtual void addGLObjects(std::vector<GLObject*> &list);
    
    virtual void loadPLY(PLYReader &ply);
    virtual void savePLY(const char *name, bool all=true, ply_encoding enc=ply_binary) const;
};

}

#endif
