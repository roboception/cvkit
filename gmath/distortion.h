/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2015, Institute of Robotics and Mechatronics, German Aerospace Center
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

#ifndef GMATH_DISTORTION_H
#define GMATH_DISTORTION_H

#include <gutil/properties.h>

namespace gmath
{

/**
  Base class for modelling lens distortion. The base class implements a dummy
  that does nothing.
*/

class Distortion
{
  public:
    
    virtual ~Distortion();
    
    /**
      Creates and initializes the appropriate distortion model from the given
      parameters.
      
      @param prop Camera parameters.
      @param id   Camera id or -1.
      @return     Pointer to created distortion model. The pointer must be deleted
                  by the calling function.
    */
    
    static Distortion *create(const gutil::Properties &prop, int id=-1);
    
    /**
      Clones the current object.
      
      @return Pointer to clone object. The pointer must be deleted by the
              calling function.
    */
    
    virtual Distortion *clone() const;
    
    virtual int countParameter() const;
    virtual double getParameter(int i) const;
    virtual void setParameter(int i, double v);
    virtual void transform(double &x, double &y, double xd, double yd) const;
    virtual void invTransform(double &xd, double &yd, double x, double y) const;
    
    /**
      Stores the parameters of the distortion model in the provided property
      object.
      
      @param prop Camera parameters.
      @param id   Camera id or -1.
    */
    
    virtual void getProperties(gutil::Properties &prop, int id=-1) const;
};

/**
  Class for modelling radial lens distortion.
*/

class RadialDistortion : public Distortion
{
  public:
    
    RadialDistortion(int n);
    RadialDistortion(const gutil::Properties &prop, int id=-1);
    
    Distortion *clone() const;
    
    int countParameter() const;
    double getParameter(int i) const;
    void setParameter(int i, double v);
    void transform(double &x, double &y, double xd, double yd) const;
    void invTransform(double &xd, double &yd, double x, double y) const;
    void getProperties(gutil::Properties &prop, int id=-1) const;
  
  private:
  
    int kn;
    double kd[3];
};

/**
  Class for modelling radial and tangential lens distortion.
*/

class RadialTangentialDistortion : public Distortion
{
  public:
    
    RadialTangentialDistortion(int n);
    RadialTangentialDistortion(const gutil::Properties &prop, int id=-1);
    
    Distortion *clone() const;
    
    int countParameter() const;
    double getParameter(int i) const;
    void setParameter(int i, double v);
    void transform(double &x, double &y, double xd, double yd) const;
    void invTransform(double &xd, double &yd, double x, double y) const;
    void getProperties(gutil::Properties &prop, int id=-1) const;
  
  private:
  
    int kn;
    double kd[5];
};

/**
  Class for modelling equidistant lens distortion.
*/

class EquidistantDistortion : public Distortion
{
  public:
    
    EquidistantDistortion();
    EquidistantDistortion(const gutil::Properties &prop, int id=-1);
    
    Distortion *clone() const;
    
    int countParameter() const;
    double getParameter(int i) const;
    void setParameter(int i, double v);
    void transform(double &x, double &y, double xd, double yd) const;
    void invTransform(double &xd, double &yd, double x, double y) const;
    void getProperties(gutil::Properties &prop, int id=-1) const;
  
  private:
  
    double ed[4];
};

}

#endif
