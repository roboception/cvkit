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

#ifndef GVR_TEXTUREDMESH_H
#define GVR_TEXTUREDMESH_H

#include "mesh.h"

#include <string>

namespace gvr
{

/**
  This class implements a mesh with texture coordinates for each vertex. See
  the MultiTexturedMesh for a mesh with multiple texture images with texture
  coordinates for each corner of each triangle.
*/

class MultiTexturedMesh;

class TexturedMesh : public Mesh
{
  private:
  
    std::string basepath;
    std::string name;
    float       *uv;
  
    TexturedMesh(const TexturedMesh &);
    TexturedMesh& operator=(const TexturedMesh &);
    
  public:
    
    TexturedMesh();
    TexturedMesh(const MultiTexturedMesh &p, int t, const std::vector<bool> &vused, const std::vector<bool> &tused);
    virtual ~TexturedMesh();
    
    virtual void resizeVertexList(int vn, bool with_scanprop, bool with_scanpos);
    
    const std::string &getBasePath() const { return basepath; }
    void setBasePath(std::string s) { basepath=s; }
    
    const std::string &getTextureName() const { return name; }
    void setTextureName(std::string s) { name=s; }
    
      // texture coordinates for each vertex
    
    float *getTextureCoordArray() { return uv; }
    float getTextureCoordComp(int i, int k) const { return uv[2*i+k]; }
    void setTextureCoordComp(int i, int k, float c) { uv[2*i+k]=c; }
    
    virtual void addGLObjects(std::vector<GLObject*> &list);
    
    virtual void loadPLY(PLYReader &ply);
    virtual void savePLY(const char *name, bool all=true, ply_encoding enc=ply_binary) const;
};

}

#endif
