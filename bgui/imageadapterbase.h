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

#ifndef BGUI_IMAGEADAPTERBASE_H
#define BGUI_IMAGEADAPTERBASE_H

#include <gimage/image.h>
#include <gmath/smatrix.h>
#include <gmath/svector.h>

#include <string>

namespace bgui
{

/**
 * Base class for classes that adapt an image of different types to a color
 * image of byte type. The class supports various operations that are applied
 * an the fly, when the image data is accessed.
 */

enum mapping {map_raw, map_jet, map_rainbow};

class ImageAdapterBase
{
  protected:
  
    int  channel;
    int  rotation;
    bool flip;
    gmath::SMatrix<long, 2, 3> R;
    double scale;
    double imin, imax;
    double gamma;
    mapping map;
  
  public:
  
    ImageAdapterBase()
    {
      channel=-1;
      rotation=0;
      scale=1;
      imin=0;
      imax=255;
      gamma=1;
      map=map_raw;
      
      setRotationFlip(0, false);
    }
    
    virtual ~ImageAdapterBase() {};
    
    void setChannel(int c) { channel=c; }
    int getChannel() { return channel; }
    
    void setRotationFlip(int r, bool f)
    {
      rotation=(r & 3);
      flip=f;
      
      R=0;
      
      switch (rotation)
      {
        default:
        case 0:
            R(0, 0)=1;
            R(1, 1)=1;
            
            if (flip)
            {
              R(0, 0)=-1;
              R(0, 2)=getOriginalWidth()-1;
            }
            break;
        
        case 1:
            R(0, 1)=1;
            R(1, 0)=-1;
            R(1, 2)=getOriginalHeight()-1;
            
            if (flip)
            {
              R(1, 0)=1;
              R(1, 2)=0;
            }
            break;
        
        case 2:
            R(0, 0)=-1;
            R(0, 2)=getOriginalWidth()-1;
            R(1, 1)=-1;
            R(1, 2)=getOriginalHeight()-1;
            
            if (flip)
            {
              R(0, 0)=1;
              R(0, 2)=0;
            }
            break;
        
        case 3:
            R(0, 1)=-1;
            R(0, 2)=getOriginalWidth()-1;
            R(1, 0)=1;
            
            if (flip)
            {
              R(1, 0)=-1;
              R(1, 2)=getOriginalHeight()-1;
            }
            break;
      }
    }
    
    int getRotation() const { return rotation; }
    int getFlip() const { return flip; }
    const gmath::SMatrix<long, 2, 3> &getRotationMatrix() const { return R; }
    
    void setScale(double s) { scale=s; }
    double getScale() const { return scale; }
    
    void setMinIntensity(double v) { imin=v; }
    double getMinIntensity() const { return imin; }
    
    void setMaxIntensity(double v) { imax=v; }
    double getMaxIntensity() const { return imax; }
    
    void setGamma(double g) { gamma=g; }
    double getGamma() const { return gamma; }
    
    void setMapping(mapping m) { map=m; }
    mapping getMapping() const { return map; }
    
    virtual void setSmoothing(bool tf)=0;
    virtual bool getSmoothing()=0;
    
    virtual void adaptMinMaxIntensity(long x, long y, long w, long h)=0;
    
      // returns properties of the underlying image
    
    virtual long getOriginalWidth() const=0;
    virtual long getOriginalHeight() const=0;
    virtual int getOriginalDepth() const=0;
    virtual std::string getOriginalType() const=0;
    virtual double getIntensityOfPixel(long x, long y) const=0;
    virtual std::string getDescriptionOfPixel(long x, long y) const=0;
    
      // size of scaled image
    
    virtual long getWidth() const=0;
    virtual long getHeight() const=0;
    
      // converts a part into the given rgb image,
      // the part can be partly or fully out of bounds
    
    virtual void copyInto(gimage::ImageU8 &rgb, long x=0, long y=0) const=0;
};

}

#endif

