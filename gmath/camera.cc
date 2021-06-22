/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2016 Roboception GmbH
 * Copyright (c) 2014 Institute of Robotics and Mechatronics, German Aerospace Center
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

#include "camera.h"
#include "minpack.h"
#include "polynomial.h"

#include "linalg.h"

#include <limits>
#include <cstring>

namespace gmath
{

std::string Camera::getCameraKey(const char *key, int id) const
{
  std::ostringstream os;

  os << "camera.";

  if (id >= 0)
  {
    os << id << '.';
  }

  os << key;

  return os.str();
}

Camera::Camera(const gutil::Properties &prop, int id)
{
  prop.getValue(getCameraKey("R", id).c_str(), R, "[1 0 0; 0 1 0; 0 0 1]");
  prop.getValue(getCameraKey("T", id).c_str(), T, "[0 0 0]");
  prop.getValue(getCameraKey("width", id).c_str(), width, "0");
  prop.getValue(getCameraKey("height", id).c_str(), height, "0");

  prop.getValue(getCameraKey("zmin", id).c_str(), zmin, "0");
  prop.getValue(getCameraKey("zmax", id).c_str(), zmax, "-1");

  if (zmax < 0)
  {
    zmax=std::numeric_limits<double>::max();
  }

  prop.getStringVector(getCameraKey("match", id).c_str(), match, "", ',');
}

bool Camera::isInside(const Vector2d &p) const
{
  return p[0] >= 0 && p[0] < width && p[1] >= 0 && p[1] < height;
}

bool Camera::isInside(const Vector3d &p) const
{
  return p[0] >= 0 && p[0] < width && p[1] >= 0 && p[1] < height &&
         !std::isfinite(p[2]);
}

void Camera::setDownscaled(int ds)
{
  if (ds > 1)
  {
    width=(width+ds-1)/ds;
    height=(height+ds-1)/ds;
  }
}

void Camera::setPart(long x, long y, long w, long h)
{
  width=w;
  height=h;
}

void Camera::getProperties(gutil::Properties &prop, int id) const
{
  prop.putValue(getCameraKey("R", id).c_str(), R);
  prop.putValue(getCameraKey("T", id).c_str(), T);

  if (width != 0)
  {
    prop.putValue(getCameraKey("width", id).c_str(), width);
  }

  if (height != 0)
  {
    prop.putValue(getCameraKey("height", id).c_str(), height);
  }

  if (zmin != 0 || zmax != std::numeric_limits<double>::max())
  {
    prop.putValue(getCameraKey("zmin", id).c_str(), zmin);
    prop.putValue(getCameraKey("zmax", id).c_str(), zmax);
  }

  if (match.size() > 0)
  {
    prop.putStringVector(getCameraKey("match", id).c_str(), match, ',');
  }
}

PinholeCamera::PinholeCamera()
{
  rho=0;
  dist=0;
}

PinholeCamera::PinholeCamera(const PinholeCamera &pc) : Camera(pc), A(pc.A)
{
  rho=pc.rho;

  dist=0;

  if (pc.dist != 0)
  {
    dist=pc.dist->clone();
  }
}

PinholeCamera::PinholeCamera(const gutil::Properties &prop, int id)
  : Camera(prop, id)
{
  Matrix33d P;
  prop.getValue(getCameraKey("A", id).c_str(), P);

  std::string origin;
  prop.getValue("origin", origin, "corner");

  if (origin == "center")
  {
    P(0, 2)+=0.5;
    P(1, 2)+=0.5;
  }

  double dx, dy;
  prop.getValue(getCameraKey("dx", id).c_str(), dx, 0.0);
  prop.getValue(getCameraKey("dy", id).c_str(), dy, 0.0);

  P(0, 2)+=dx;
  P(1, 2)+=dy;

  setA(P);

  prop.getValue(getCameraKey("rho", id).c_str(), rho, "0");

  if (rho == 0)
  {
    prop.getValue("rho", rho, "0");
  }

  if (rho == 0)
  {
    double f, t;
    prop.getValue("f", f, "0");
    prop.getValue("t", t, "0");
    rho=f*t;
  }

  dist=Distortion::create(prop, id);
}

PinholeCamera &PinholeCamera::operator = (const PinholeCamera &pc)
{
  if (this != &pc)
  {
    Camera::operator=(pc);

    A=pc.A;
    rho=pc.rho;

    delete dist;
    dist=0;

    if (pc.dist != 0)
    {
      dist=pc.dist->clone();
    }
  }

  return *this;
}

Camera *PinholeCamera::clone() const
{
  return new PinholeCamera(*this);
}

void PinholeCamera::setA(const Matrix33d &AA)
{
  A=AA;

  if (A(1, 0) != 0 || A(2, 0) != 0 || A(2, 1) != 0 || A(2, 2) != 1)
  {
    std::ostringstream out;
    out << "Invalid camera matrix: " << A;
    throw gutil::IOException(out.str().c_str());
  }
}

void PinholeCamera::setDownscaled(int ds)
{
  Camera::setDownscaled(ds);

  if (ds > 1)
  {
    A/=ds;
    A(2, 2)=1.0;
    rho/=ds;
  }
}

void PinholeCamera::setPart(long x, long y, long w, long h)
{
  Camera::setPart(x, y, w, h);

  A(0, 2)-=x;
  A(1, 2)-=y;
}

void PinholeCamera::getProperties(gutil::Properties &prop, int id) const
{
  Camera::getProperties(prop, id);

  prop.putValue(getCameraKey("A", id).c_str(), A);
  prop.putValue(getCameraKey("rho", id).c_str(), rho);

  if (dist != 0)
  {
    dist->getProperties(prop, id);
  }
}

double PinholeCamera::projectPoint(Vector2d &p, const Vector3d &Pw) const
{
  // transform into camera coordiante system

  Vector3d Pc=transpose(getR())*(Pw-getT());

  // if rho is given, then compute disparity and determine if the point
  // is behind the camera

  double d=std::numeric_limits<double>::infinity();

  if (rho != 0)
  {
    d=rho/Pc[2];
  }

  if (Pc[2] <= 0)
  {
    d=-1;
  }

  // apply lens distortion

  Pc/=Pc[2];

  if (dist != 0)
  {
    const double x=Pc[0];
    const double y=Pc[1];
    dist->transform(Pc[0], Pc[1], x, y);
  }

  // apply camera matrix

  p[0]=A(0, 0)*Pc[0]+A(0, 1)*Pc[1]+A(0, 2);
  p[1]=A(1, 1)*Pc[1]+A(1, 2);

  return d;
}

void PinholeCamera::reconstructPoint(Vector3d &Pw, const Vector2d &p, double d)
const
{
  if (rho == 0)
  {
    throw gutil::IOException("Cannot reconstruct point with unknown rho");
  }

  assert(std::isfinite(d));

  Vector3d Pc;
  reconstructLocal(Pc, p);

  Pc*=rho/d;

  Pw=getR()*Pc+getT();
}

void PinholeCamera::reconstructRay(Vector3d &V, Vector3d &C, const Vector2d &p) const
{
  Vector3d Pc;
  reconstructLocal(Pc, p);

  V=getR()*Pc;
  C=getT();
}

void PinholeCamera::projectPointLocal(Vector2d &p, const Vector3d &Pc) const
{
  // apply lens distortion

  Vector3d P=Pc/Pc[2];

  if (dist != 0)
  {
    const double x=P[0];
    const double y=P[1];
    dist->transform(P[0], P[1], x, y);
  }

  // apply camera matrix

  p[0]=A(0, 0)*P[0]+A(0, 1)*P[1]+A(0, 2);
  p[1]=A(1, 1)*P[1]+A(1, 2);
}

void PinholeCamera::reconstructLocal(Vector3d &q, const Vector2d &p) const
{
  // apply inverse camera matrix

  q[0]=p[0]/A(0, 0)-p[1]*A(0, 1)/(A(0, 0)*A(1, 1))+
       (A(0, 1)*A(1, 2)-A(1, 1)*A(0, 2))/
       (A(0, 0)*A(1, 1)*A(2, 2));
  q[1]=p[1]/A(1, 1)-A(1, 2)/(A(2, 2)*A(1, 1));
  q[2]=1;

  // apply inverse lens distortion

  if (dist != 0)
  {
    const double x=q[0];
    const double y=q[1];
    dist->invTransform(q[0], q[1], x, y);
  }
}

OrthoCamera::OrthoCamera(const gutil::Properties &prop)
  : Camera(prop, -1)
{
  prop.getValue("resolution", res);
  prop.getValue("depth.resolution", dres, "1");

  if (prop.contains("origin.T"))
  {
    Vector3d T;
    prop.getValue("origin.T", T);

    std::string origin;
    prop.getValue("origin", origin, "corner");

    if (origin == "center")
    {
      T[0]-=res/2;
      T[1]+=res/2;
    }

    setT(T);
  }
}

Camera *OrthoCamera::clone() const
{
  OrthoCamera *ret=new OrthoCamera();

  *ret=*this;

  return ret;
}

void OrthoCamera::setDownscaled(int ds)
{
  Camera::setDownscaled(ds);

  if (ds > 1)
  {
    res*=ds;
    dres*=ds;
  }
}

void OrthoCamera::setPart(long x, long y, long w, long h)
{
  Camera::setPart(x, y, w, h);

  Vector3d T=getT();

  T[0]+=x*res;
  T[1]-=y*res;

  setT(T);
}

void OrthoCamera::getProperties(gutil::Properties &prop, int id) const
{
  Camera::getProperties(prop, -1);

  prop.putValue("resolution", res);
  prop.putValue("depth.resolution", dres);
}

double OrthoCamera::projectPoint(Vector2d &p, const Vector3d &Pw) const
{
  Vector3d Pc=transpose(getR())*(Pw-getT());

  p[0]=Pc[0]/res;
  p[1]=-Pc[1]/res;

  return Pc[2]/dres;
}

void OrthoCamera::reconstructPoint(Vector3d &Pw, const Vector2d &p, double d)
const
{
  if (!std::isfinite(d))
  {
    throw gutil::InvalidArgumentException("Cannot reconstruct invalid point");
  }

  Vector3d Pc;

  Pc[0]=p[0]*res;
  Pc[1]=-p[1]*res;
  Pc[2]=d*dres;

  Pw=getR()*Pc+getT();
}

void OrthoCamera::reconstructRay(Vector3d &V, Vector3d &C, const Vector2d &p)
const
{
  V=getR()*Vector3d(0, 0, -1);

  C[0]=p[0]*res;
  C[1]=-p[1]*res;
  C[2]=0;

  C+=getT();
}

}
