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

#ifndef GMATH_CAMERA_H
#define GMATH_CAMERA_H

#include "smatrix.h"
#include "svector.h"
#include "distortion.h"

#include <gutil/properties.h>
#include <gutil/exception.h>

#include <string>
#include <vector>
#include <limits>

#ifdef WIN32
#undef min
#undef max
#endif

namespace gmath
{

/*
  Virtual base class with extrinsic parameters and image size.
*/

class Camera
{
  private:

    std::string              name;
    long                     width, height;
    Matrix33d                R;
    Vector3d                 T;
    double                   zmin, zmax;
    std::vector<std::string> match;

  protected:

    std::string getCameraKey(const char *key, int id) const;

  public:

    Camera() { width=0; height=0; zmin=0; zmax=std::numeric_limits<double>::max(); };
    Camera(const gutil::Properties &prop, int id=-1);
    virtual ~Camera() {}

    virtual Camera *clone() const=0;

    void setName(const std::string &s) { name=s; }
    const std::string &getName() const { return name; }

    void setSize(long w, long h) { width=w; height=h; }
    long getWidth() const { return width; }
    long getHeight() const { return height; }

    // Pw=R*Pc+T

    void setR(const Matrix33d &RR) { R=RR; }
    void setT(const Vector3d &TT) { T=TT; }
    const Matrix33d &getR() const { return R; }
    const Vector3d &getT() const { return T; }

    void setZMin(double z) { zmin=z; }
    void setZMax(double z) { zmax=z; }
    double getZMin() const { return zmin; }
    double getZMax() const { return zmax; }

    void addMatch(std::string s) { match.push_back(s); }
    int countMatch() { return static_cast<int>(match.size()); }
    const std::string &getMatch(int i) { return match[i]; }

    bool isInside(const Vector2d &p) const;
    bool isInside(const Vector3d &p) const;

    virtual bool isPerspective() const { return false; }
    virtual void setDownscaled(int ds);
    virtual void setPart(long x, long y, long w, long h);

    virtual void getProperties(gutil::Properties &prop, int id=-1) const;

    // transformation between a point P=[x y z] of the scene and p=[i k]
    // in the image, with i as image column, k as image row and d as depth
    // encoding, which can be a disparity, depending on the sub-class

    virtual double projectPoint(Vector2d &p, const Vector3d &Pw) const=0;
    virtual void reconstructPoint(Vector3d &Pw, const Vector2d &p, double d) const=0;

    // reconstruction of the raw from the camera C into scene with direction
    // V, from a point in the image p=[i k]

    virtual void reconstructRay(Vector3d &V, Vector3d &C, const Vector2d &p) const=0;
};

/*
  Pinhole camera in which all rays of light pass through the center of
  projection.
*/

class PinholeCamera : public Camera
{
  private:

    Matrix33d A;
    double    rho;

    Distortion *dist;

  public:

    PinholeCamera();
    PinholeCamera(const PinholeCamera &pc);
    PinholeCamera(const gutil::Properties &prop, int id=-1);
    virtual ~PinholeCamera() { delete dist; };

    PinholeCamera &operator = (const PinholeCamera &pc);
    virtual Camera *clone() const;

    void setA(const Matrix33d &AA);
    const Matrix33d &getA() const { return A; }
    double getMeanFocalLength() const { return (A(0, 0)+A(1, 1))/2; }

    void setRho(double r) { rho=r; }
    double getRho() const { return rho; }

    void setDistortion(Distortion *d)
    {
      delete dist;
      dist=0;

      if (d != 0)
      {
        dist=d->clone();
      }
    }

    const Distortion *getDistortion() const { return dist; }

    virtual bool isPerspective() const { return true; }
    virtual void setDownscaled(int ds);
    virtual void setPart(long x, long y, long w, long h);

    virtual void getProperties(gutil::Properties &prop, int id=-1) const;
    virtual double projectPoint(Vector2d &p, const Vector3d &Pw) const;
    virtual void reconstructPoint(Vector3d &Pw, const Vector2d &p, double d) const;
    virtual void reconstructRay(Vector3d &V, Vector3d &C, const Vector2d &p) const;

    // projection and reconstruction of a point in the local camera
    // coordinate system

    void projectPointLocal(Vector2d &p, const Vector3d &Pc) const;
    void reconstructLocal(Vector3d &q, const Vector2d &p) const;
};

/*
  Orthogonal camera in which all rays of light are parallel and orthogonal to
  the image.
*/

class OrthoCamera : public Camera
{
  private:

    double res;
    double dres;

  public:

    OrthoCamera() { res=1; dres=1; };
    OrthoCamera(const gutil::Properties &prop);
    virtual ~OrthoCamera() {};

    virtual Camera *clone() const;

    void setResolution(double r) { res=r; }
    double getResolution() const { return res; }

    void setDepthResolution(double r) { dres=r; }
    double getDepthResolution() const { return dres; }

    virtual void setDownscaled(int ds);
    virtual void setPart(long x, long y, long w, long h);

    virtual void getProperties(gutil::Properties &prop, int id=-1) const;
    virtual double projectPoint(Vector2d &p, const Vector3d &Pw) const;
    virtual void reconstructPoint(Vector3d &Pw, const Vector2d &p, double d) const;
    virtual void reconstructRay(Vector3d &V, Vector3d &C, const Vector2d &p) const;
};

}

#endif
