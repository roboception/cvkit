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

#ifndef GVR_MODEL_H
#define GVR_MODEL_H

#include "ply.h"

#include <gmath/svector.h>
#include <gutil/exception.h>
#include <gmath/svector.h>
#include <gmath/smatrix.h>
#include <gmath/camera.h>

#include <vector>

namespace gvr
{

const int ID_CAMERA_BODY=0;
const int ID_CAMERA_RANGE=1;
const int ID_CAMERA_LINK=2;
const int ID_MODEL_START=3;
    
class GLObject;

class Model
{
  private:
  
    int              id;
    gmath::Vector3d  origin;
    
    gmath::Matrix33d Rc;
    gmath::Vector3d  Tc;
    
  protected:
  
    void setOriginFromPLY(PLYReader &ply);
    void setOriginToPLY(PLYWriter &ply) const;
  
  public:
    
    Model()
    {
      id=0;
      Rc=0;
    }
    
    Model(const Model &p)
    {
      id=p.getID();
      origin=p.getOrigin();
    }
    
    virtual ~Model() {};
    
    int getID() const { return id; }
    void setID(int i) { id=i; }
    
    const gmath::Vector3d &getOrigin() const { return origin; }
    void setOrigin(const gmath::Vector3d &v) { origin=v; }
    
    virtual void translate(const gmath::Vector3d &v)
    {
      origin+=v;
      setDefCameraRT(getDefCameraR(), getDefCameraT()+v);
    }
    
    const gmath::Matrix33d &getDefCameraR() const { return Rc; }
    const gmath::Vector3d &getDefCameraT() const { return Tc; }
    void setDefCameraRT(const gmath::Matrix33d &R, const gmath::Vector3d &T) { Rc=R; Tc=T; }
    
    virtual void addExtend(gmath::Vector3d &min, gmath::Vector3d &max) const=0;
    
    virtual void addGLObjects(std::vector<GLObject*> &list)=0;

    virtual void loadPLY(PLYReader &ply)=0;
    virtual void savePLY(const char *name, bool all=true, ply_encoding enc=ply_binary) const=0;
};

class FloatArrayReceiver : public PLYReceiver
{
  private:
    
    float  *array;
    size_t size;
    size_t offset;
  
  public:
  
    FloatArrayReceiver(float *_array, size_t _size, size_t _offset);
    void setValue(int instance, const PLYValue &value);
};

class UInt8Receiver : public PLYReceiver
{
  private:
    
    unsigned char *array;
    size_t        offset;
    float         scale;
  
  public:
  
    UInt8Receiver(unsigned char *_array, int _offset, float _scale);
    void setValue(int instance, const PLYValue &value);
};

// Loads a model from ply or from depth image

Model *loadModel(const char *name, const char *spath=0, bool verbose=false);

// Loads a model from ply

Model *loadPLY(const char *name);

// Loads a model from depth image

Model *loadDepth(const char *name, const char *spath=0, bool verbose=false);

}

#endif
