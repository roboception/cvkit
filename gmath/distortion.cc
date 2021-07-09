/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2016, 2019 Roboception GmbH
 * Copyright (c) 2015 Institute of Robotics and Mechatronics, German Aerospace Center
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

#include "distortion.h"

#include <gmath/minpack.h>

#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
#include <algorithm>

namespace gmath
{

namespace
{

std::string getCameraKey(const char *key, int id)
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

}

// --------------------- Distortion ---------------------

Distortion::~Distortion()
{ }

Distortion *Distortion::create(const gutil::Properties &prop, int id)
{
  // try equidistant model

  EquidistantDistortion ed(prop, id);

  if (ed.getParameter(0) != 0 || ed.getParameter(1) != 0 ||
      ed.getParameter(2) != 0 || ed.getParameter(3) != 0)
  {
    return ed.clone();
  }

  // create best fitting model, i.e. with lowest number of parameters

  RationalTangentialThinPrismDistortion ratpd(prop, id);

  if (ratpd.getParameter(8) != 0 || ratpd.getParameter(9) != 0 || ratpd.getParameter(10) != 0 ||
      ratpd.getParameter(11) != 0)
  {
    return ratpd.clone();
  }

  RationalTangentialDistortion ratd(prop, id);

  if (ratd.getParameter(5) != 0 || ratd.getParameter(6) != 0 || ratd.getParameter(7) != 0)
  {
    return ratd.clone();
  }

  RadialTangentialDistortion rtd(prop, id);

  if (rtd.getParameter(0) != 0 || rtd.getParameter(1) != 0)
  {
    return rtd.clone();
  }

  if (rtd.countParameter() > 2 || rtd.getParameter(0) != 0 || rtd.getParameter(1) != 0)
  {
    return new RadialDistortion(prop, id);
  }

  return 0;
}

void Distortion::cleanAllProperties(gutil::Properties &prop, int id)
{
  EquidistantDistortion ed;
  RadialTangentialDistortion rtd(3);
  RadialDistortion rd(3);

  ed.cleanProperties(prop, id);
  rtd.cleanProperties(prop, id);
  rd.cleanProperties(prop, id);
}

Distortion *Distortion::clone() const
{
  return new Distortion();
}

int Distortion::countParameter() const
{
  return 0;
}

double Distortion::getParameter(int i) const
{
  return 0;
}

void Distortion::setParameter(int i, double v)
{ }

void Distortion::transform(double &x, double &y, double xd, double yd) const
{
  x=xd;
  y=yd;
}

void Distortion::invTransform(double &xd, double &yd, double x, double y) const
{
  xd=x;
  yd=y;
}

void Distortion::getProperties(gutil::Properties &prop, int id) const
{ }

void Distortion::cleanProperties(gutil::Properties &prop, int id) const
{ }

// --------------------- RadialDistortion ---------------------

RadialDistortion::RadialDistortion(int n)
{
  n=std::min(n, 3);

  kn=n;
  kd[0]=0;
  kd[1]=0;
  kd[2]=0;
}

RadialDistortion::RadialDistortion(const gutil::Properties &prop, int id)
{
  prop.getValue(getCameraKey("k1", id).c_str(), kd[0], "0");
  prop.getValue(getCameraKey("k2", id).c_str(), kd[1], "0");
  prop.getValue(getCameraKey("k3", id).c_str(), kd[2], "0");

  kn=0;

  if (kd[2] != 0)
  {
    kn++;
  }

  if (kd[1] != 0 || kn > 0)
  {
    kn++;
  }

  if (kd[0] != 0 || kn > 0)
  {
    kn++;
  }
}

Distortion *RadialDistortion::clone() const
{
  RadialDistortion *ret=new RadialDistortion(kn);

  ret->kd[0]=kd[0];
  ret->kd[1]=kd[1];
  ret->kd[2]=kd[2];

  return ret;
}

int RadialDistortion::countParameter() const
{
  return kn;
}

double RadialDistortion::getParameter(int i) const
{
  return kd[i];
}

void RadialDistortion::setParameter(int i, double v)
{
  kd[i]=v;
}

void RadialDistortion::transform(double &x, double &y, double xd, double yd) const
{
  const double r2=xd*xd+yd*yd;
  const double s=1.0+kd[0]*r2+kd[1]*r2*r2+kd[2]*r2*r2*r2;

  x=xd*s;
  y=yd*s;
}

namespace
{

struct RadialDistortionParameter
{
  double kd[3];
  double xd, yd;
};

int computeRadialDistortion(int n, double x[], int m, double fvec[],
                            double fjac[], void *up)
{
  RadialDistortionParameter *p=static_cast<RadialDistortionParameter *>(up);

  if (fvec != 0)
  {
    const double r2=x[0]*x[0]+x[1]*x[1];
    const double s=1.0+p->kd[0]*r2+p->kd[1]*r2*r2+p->kd[2]*r2*r2*r2;

    fvec[0]=p->xd-x[0]*s;
    fvec[1]=p->yd-x[1]*s;
  }

  if (fjac != 0)
  {
    const double r2=x[0]*x[0]+x[1]*x[1];

    const double f=1.0+p->kd[0]*r2+p->kd[1]*r2*r2+p->kd[2]*r2*r2*r2;
    const double fs=p->kd[0]+2*p->kd[1]*r2+3*p->kd[2]*r2*r2;

    fjac[0]=-(f+2*x[0]*x[0]*fs);
    fjac[1]=-(2*x[0]*x[1]*fs);
    fjac[2]=fjac[1];
    fjac[3]=-(f+2*x[1]*x[1]*fs);
  }

  return 0;
}

}

void RadialDistortion::invTransform(double &xd, double &yd, double xx, double yy) const
{
  double x[2]= {xx, yy};

  RadialDistortionParameter param;

  param.kd[0]=kd[0];
  param.kd[1]=kd[1];
  param.kd[2]=kd[2];
  param.xd=xx;
  param.yd=yy;

  double fvec[2];

  long ltmp[2]= {0, 0};
  double dtmp[16];

  memset(dtmp, 0, 16*sizeof(double));
  gmath::slmder(computeRadialDistortion, 2, 2, x, fvec, &param, 1e-6, ltmp, dtmp);

  xd=x[0];
  yd=x[1];
}

void RadialDistortion::getProperties(gutil::Properties &prop, int id) const
{
  prop.putValue(getCameraKey("k1", id).c_str(), kd[0]);
  prop.putValue(getCameraKey("k2", id).c_str(), kd[1]);
  prop.putValue(getCameraKey("k3", id).c_str(), kd[2]);
}

void RadialDistortion::cleanProperties(gutil::Properties &prop, int id) const
{
  prop.remove(getCameraKey("k1", id).c_str());
  prop.remove(getCameraKey("k2", id).c_str());
  prop.remove(getCameraKey("k3", id).c_str());
}

// --------------------- RadialTangentialDistortion ---------------------

RadialTangentialDistortion::RadialTangentialDistortion(int n)
{
  n=std::min(n, 3);

  kn=n;
  kd[0]=0;
  kd[1]=0;
  kd[2]=0;
  kd[3]=0;
  kd[4]=0;
}

RadialTangentialDistortion::RadialTangentialDistortion(const gutil::Properties &prop, int id)
{
  prop.getValue(getCameraKey("p1", id).c_str(), kd[0], "0");
  prop.getValue(getCameraKey("p2", id).c_str(), kd[1], "0");

  prop.getValue(getCameraKey("k1", id).c_str(), kd[2], "0");
  prop.getValue(getCameraKey("k2", id).c_str(), kd[3], "0");
  prop.getValue(getCameraKey("k3", id).c_str(), kd[4], "0");

  kn=0;

  if (kd[4] != 0)
  {
    kn++;
  }

  if (kd[3] != 0 || kn > 0)
  {
    kn++;
  }

  if (kd[2] != 0 || kn > 0)
  {
    kn++;
  }
}

Distortion *RadialTangentialDistortion::clone() const
{
  RadialTangentialDistortion *ret=new RadialTangentialDistortion(kn);

  for (int i=0; i<5; i++)
  {
    ret->kd[i]=kd[i];
  }

  return ret;
}

int RadialTangentialDistortion::countParameter() const
{
  return kn+2;
}

double RadialTangentialDistortion::getParameter(int i) const
{
  return kd[i];
}

void RadialTangentialDistortion::setParameter(int i, double v)
{
  kd[i]=v;
}

void RadialTangentialDistortion::transform(double &x, double &y, double xd, double yd) const
{
  const double r2=xd*xd+yd*yd;
  const double s=1.0 + kd[2]*r2 + kd[3]*r2*r2 + kd[4]*r2*r2*r2;

  const double xx=xd*s + 2*kd[0]*xd*yd        +   kd[1]*(r2+2*xd*xd);
  const double yy=yd*s +   kd[0]*(r2+2*yd*yd) + 2*kd[1]*xd*yd;

  x=xx;
  y=yy;
}

namespace
{

struct RadialTangentialDistortionParameter
{
  double kd[5];
  double xd, yd;
};

int computeRadialTangentialDistortion(int n, double x[], int m, double fvec[], double fjac[],
                                      void *up)
{
  RadialTangentialDistortionParameter *p=static_cast<RadialTangentialDistortionParameter *>(up);

  if (fvec != 0)
  {
    const double r2=x[0]*x[0]+x[1]*x[1];
    const double s=1.0 + p->kd[2]*r2 + p->kd[3]*r2*r2 + p->kd[4]*r2*r2*r2;

    fvec[0]=p->xd-(x[0]*s + 2*p->kd[0]*x[0]*x[1]        +   p->kd[1]*(r2+2*x[0]*x[0]));
    fvec[1]=p->yd-(x[1]*s +   p->kd[0]*(r2+2*x[1]*x[1]) + 2*p->kd[1]*x[0]*x[1]);
  }

  if (fjac != 0)
  {
    const double r2=x[0]*x[0]+x[1]*x[1];

    const double f=1.0 + p->kd[2]*r2 +   p->kd[3]*r2*r2 +   p->kd[4]*r2*r2*r2;
    const double fs=     p->kd[2]    + 2*p->kd[3]*r2    + 3*p->kd[4]*r2*r2;

    fjac[0]=-(f+2*x[0]*x[0]*fs+2*p->kd[0]*x[1]+6*p->kd[1]*x[0]);
    fjac[1]=-(2*x[0]*x[1]*fs+2*p->kd[0]*x[0]+2*p->kd[1]*x[1]);
    fjac[2]=fjac[1];
    fjac[3]=-(f+2*x[1]*x[1]*fs+6*p->kd[0]*x[1]+2*p->kd[1]*x[0]);
  }

  return 0;
}

}

void RadialTangentialDistortion::invTransform(double &xd, double &yd, double xx, double yy) const
{
  double x[2]= {xx, yy};

  RadialTangentialDistortionParameter param;

  param.kd[0]=kd[0];
  param.kd[1]=kd[1];
  param.kd[2]=kd[2];
  param.kd[3]=kd[3];
  param.kd[4]=kd[4];

  param.xd=xx;
  param.yd=yy;

  double fvec[2];

  long ltmp[2]= {0, 0};
  double dtmp[16];

  memset(dtmp, 0, 16*sizeof(double));
  gmath::slmder(computeRadialTangentialDistortion, 2, 2, x, fvec, &param, 1e-6, ltmp, dtmp);

  xd=x[0];
  yd=x[1];
}

void RadialTangentialDistortion::getProperties(gutil::Properties &prop, int id) const
{
  prop.putValue(getCameraKey("p1", id).c_str(), kd[0]);
  prop.putValue(getCameraKey("p2", id).c_str(), kd[1]);

  prop.putValue(getCameraKey("k1", id).c_str(), kd[2]);
  prop.putValue(getCameraKey("k2", id).c_str(), kd[3]);
  prop.putValue(getCameraKey("k3", id).c_str(), kd[4]);
}

void RadialTangentialDistortion::cleanProperties(gutil::Properties &prop, int id) const
{
  prop.remove(getCameraKey("p1", id).c_str());
  prop.remove(getCameraKey("p2", id).c_str());

  prop.remove(getCameraKey("k1", id).c_str());
  prop.remove(getCameraKey("k2", id).c_str());
  prop.remove(getCameraKey("k3", id).c_str());
}

// --------------------- RationalTangentialDistortion ---------------------

RationalTangentialDistortion::RationalTangentialDistortion()
{
  for (int i=0; i<8; i++)
  {
    kd[i]=0;
  }
}

RationalTangentialDistortion::RationalTangentialDistortion(const gutil::Properties &prop, int id)
{
  prop.getValue(getCameraKey("p1", id).c_str(), kd[0], "0");
  prop.getValue(getCameraKey("p2", id).c_str(), kd[1], "0");

  prop.getValue(getCameraKey("k1", id).c_str(), kd[2], "0");
  prop.getValue(getCameraKey("k2", id).c_str(), kd[3], "0");
  prop.getValue(getCameraKey("k3", id).c_str(), kd[4], "0");
  prop.getValue(getCameraKey("k4", id).c_str(), kd[5], "0");
  prop.getValue(getCameraKey("k5", id).c_str(), kd[6], "0");
  prop.getValue(getCameraKey("k6", id).c_str(), kd[7], "0");
}

Distortion *RationalTangentialDistortion::clone() const
{
  RationalTangentialDistortion *ret=new RationalTangentialDistortion();

  for (int i=0; i<8; i++)
  {
    ret->kd[i]=kd[i];
  }

  return ret;
}

int RationalTangentialDistortion::countParameter() const
{
  return 8;
}

double RationalTangentialDistortion::getParameter(int i) const
{
  return kd[i];
}

void RationalTangentialDistortion::setParameter(int i, double v)
{
  kd[i]=v;
}

void RationalTangentialDistortion::transform(double &x, double &y, double xd, double yd) const
{
  const double r2=xd*xd+yd*yd;
  const double r4=r2*r2;
  const double r6=r4*r2;

  const double s=(1.0 + kd[2]*r2 + kd[3]*r4 + kd[4]*r6) /
                 (1.0 + kd[5]*r2 + kd[6]*r4 + kd[7]*r6);

  const double xx=xd*s + 2*kd[0]*xd*yd      + kd[1]*(r2+2*xd*xd);
  const double yy=yd*s + kd[0]*(r2+2*yd*yd) + 2*kd[1]*xd*yd;

  x=xx;
  y=yy;
}

namespace
{

struct RationalTangentialDistortionParameter
{
  double p1, p2;
  double kd[6];
  double xd, yd;
};

int computeRationalTangentialDistortion(int n, double x[], int m, double fvec[], void *up)
{
  RationalTangentialDistortionParameter *p=static_cast<RationalTangentialDistortionParameter *>(up);

  const double r2=x[0]*x[0]+x[1]*x[1];
  const double r4=r2*r2;
  const double r6=r4*r2;

  const double s=(1.0 + p->kd[0]*r2 + p->kd[1]*r4 + p->kd[2]*r6) /
                 (1.0 + p->kd[3]*r2 + p->kd[4]*r4 + p->kd[5]*r6);

  fvec[0]=p->xd - (x[0]*s + 2*p->p1*x[0]*x[1]      + p->p2*(r2+2*x[0]*x[0]));
  fvec[1]=p->yd - (x[1]*s + p->p1*(r2+2*x[1]*x[1]) + 2*p->p2*x[0]*x[1]);

  return 0;
}

}

void RationalTangentialDistortion::invTransform(double &xd, double &yd, double xx, double yy) const
{
  double x[2]= {xx, yy};

  RationalTangentialDistortionParameter param;

  param.p1=kd[0];
  param.p2=kd[1];
  param.kd[0]=kd[2];
  param.kd[1]=kd[3];
  param.kd[2]=kd[4];
  param.kd[3]=kd[5];
  param.kd[4]=kd[6];
  param.kd[5]=kd[7];
  param.xd=xx;
  param.yd=yy;

  double fvec[2];

  long ltmp[2]= {0, 0};
  double dtmp[16];

  memset(dtmp, 0, 16*sizeof(double));
  gmath::slmdif(computeRationalTangentialDistortion, 2, 2, x, fvec, &param, 1e-6, 0, ltmp, dtmp);

  xd=x[0];
  yd=x[1];
}

void RationalTangentialDistortion::getProperties(gutil::Properties &prop, int id) const
{
  prop.putValue(getCameraKey("p1", id).c_str(), kd[0]);
  prop.putValue(getCameraKey("p2", id).c_str(), kd[1]);

  prop.putValue(getCameraKey("k1", id).c_str(), kd[2]);
  prop.putValue(getCameraKey("k2", id).c_str(), kd[3]);
  prop.putValue(getCameraKey("k3", id).c_str(), kd[4]);
  prop.putValue(getCameraKey("k4", id).c_str(), kd[5]);
  prop.putValue(getCameraKey("k5", id).c_str(), kd[6]);
  prop.putValue(getCameraKey("k6", id).c_str(), kd[7]);
}

void RationalTangentialDistortion::cleanProperties(gutil::Properties &prop, int id) const
{
  prop.remove(getCameraKey("p1", id).c_str());
  prop.remove(getCameraKey("p2", id).c_str());

  prop.remove(getCameraKey("k1", id).c_str());
  prop.remove(getCameraKey("k2", id).c_str());
  prop.remove(getCameraKey("k3", id).c_str());
  prop.remove(getCameraKey("k4", id).c_str());
  prop.remove(getCameraKey("k5", id).c_str());
  prop.remove(getCameraKey("k6", id).c_str());
}

// --------------------- RationalTangentialThinPrismDistortion ---------------------

RationalTangentialThinPrismDistortion::RationalTangentialThinPrismDistortion()
{
  for (int i=0; i<12; i++)
  {
    kd[i]=0;
  }
}

RationalTangentialThinPrismDistortion::RationalTangentialThinPrismDistortion(const gutil::Properties &prop, int id)
{
  prop.getValue(getCameraKey("p1", id).c_str(), kd[0], "0");
  prop.getValue(getCameraKey("p2", id).c_str(), kd[1], "0");

  prop.getValue(getCameraKey("k1", id).c_str(), kd[2], "0");
  prop.getValue(getCameraKey("k2", id).c_str(), kd[3], "0");
  prop.getValue(getCameraKey("k3", id).c_str(), kd[4], "0");
  prop.getValue(getCameraKey("k4", id).c_str(), kd[5], "0");
  prop.getValue(getCameraKey("k5", id).c_str(), kd[6], "0");
  prop.getValue(getCameraKey("k6", id).c_str(), kd[7], "0");

  prop.getValue(getCameraKey("s1", id).c_str(), kd[8], "0");
  prop.getValue(getCameraKey("s2", id).c_str(), kd[9], "0");
  prop.getValue(getCameraKey("s3", id).c_str(), kd[10], "0");
  prop.getValue(getCameraKey("s4", id).c_str(), kd[11], "0");
}

Distortion *RationalTangentialThinPrismDistortion::clone() const
{
  RationalTangentialThinPrismDistortion *ret=new RationalTangentialThinPrismDistortion();

  for (int i=0; i<12; i++)
  {
    ret->kd[i]=kd[i];
  }

  return ret;
}

int RationalTangentialThinPrismDistortion::countParameter() const
{
  return 12;
}

double RationalTangentialThinPrismDistortion::getParameter(int i) const
{
  return kd[i];
}

void RationalTangentialThinPrismDistortion::setParameter(int i, double v)
{
  kd[i]=v;
}

void RationalTangentialThinPrismDistortion::transform(double &x, double &y, double xd, double yd) const
{
  const double r2=xd*xd+yd*yd;
  const double r4=r2*r2;
  const double r6=r4*r2;

  const double s=(1.0 + kd[2]*r2 + kd[3]*r4 + kd[4]*r6) /
                 (1.0 + kd[5]*r2 + kd[6]*r4 + kd[7]*r6);

  const double xx=xd*s + 2*kd[0]*xd*yd        +   kd[1]*(r2+2*xd*xd) + kd[8]*r2 + kd[9]*r4;
  const double yy=yd*s +   kd[0]*(r2+2*yd*yd) + 2*kd[1]*xd*yd        + kd[10]*r2 + kd[11]*r4;

  x=xx;
  y=yy;
}

namespace
{

struct RationalTangentialThinPrismDistortionParameter
{
  double p1, p2;
  double kd[6];
  double s[4];
  double xd, yd;
};

int computeRationalTangentialThinPrismDistortion(int n, double x[], int m, double fvec[], void *up)
{
  RationalTangentialThinPrismDistortionParameter *p=static_cast<RationalTangentialThinPrismDistortionParameter *>(up);

  const double r2=x[0]*x[0]+x[1]*x[1];
  const double r4=r2*r2;
  const double r6=r4*r2;

  const double s=(1.0 + p->kd[0]*r2 + p->kd[1]*r4 + p->kd[2]*r6) /
                 (1.0 + p->kd[3]*r2 + p->kd[4]*r4 + p->kd[5]*r6);

  fvec[0]=p->xd - (x[0]*s + 2*p->p1*x[0]*x[1]        +   p->p2*(r2+2*x[0]*x[0]) + p->s[0]*r2 + p->s[1]*r4);
  fvec[1]=p->yd - (x[1]*s +   p->p1*(r2+2*x[1]*x[1]) + 2*p->p2*x[0]*x[1]        + p->s[2]*r2 + p->s[3]*r4);

  return 0;
}

}

void RationalTangentialThinPrismDistortion::invTransform(double &xd, double &yd, double xx, double yy) const
{
  double x[2]= {xx, yy};

  RationalTangentialThinPrismDistortionParameter param;

  param.p1=kd[0];
  param.p2=kd[1];

  param.kd[0]=kd[2];
  param.kd[1]=kd[3];
  param.kd[2]=kd[4];
  param.kd[3]=kd[5];
  param.kd[4]=kd[6];
  param.kd[5]=kd[7];

  param.s[0]=kd[8];
  param.s[1]=kd[9];
  param.s[2]=kd[10];
  param.s[3]=kd[11];

  param.xd=xx;
  param.yd=yy;

  double fvec[2];

  long ltmp[2]= {0, 0};
  double dtmp[16];

  memset(dtmp, 0, 16*sizeof(double));
  gmath::slmdif(computeRationalTangentialThinPrismDistortion, 2, 2, x, fvec, &param, 1e-6, 0, ltmp, dtmp);

  xd=x[0];
  yd=x[1];
}

void RationalTangentialThinPrismDistortion::getProperties(gutil::Properties &prop, int id) const
{
  prop.putValue(getCameraKey("p1", id).c_str(), kd[0]);
  prop.putValue(getCameraKey("p2", id).c_str(), kd[1]);

  prop.putValue(getCameraKey("k1", id).c_str(), kd[2]);
  prop.putValue(getCameraKey("k2", id).c_str(), kd[3]);
  prop.putValue(getCameraKey("k3", id).c_str(), kd[4]);
  prop.putValue(getCameraKey("k4", id).c_str(), kd[5]);
  prop.putValue(getCameraKey("k5", id).c_str(), kd[6]);
  prop.putValue(getCameraKey("k6", id).c_str(), kd[7]);

  prop.putValue(getCameraKey("s1", id).c_str(), kd[8]);
  prop.putValue(getCameraKey("s2", id).c_str(), kd[9]);
  prop.putValue(getCameraKey("s3", id).c_str(), kd[10]);
  prop.putValue(getCameraKey("s4", id).c_str(), kd[11]);
}

void RationalTangentialThinPrismDistortion::cleanProperties(gutil::Properties &prop, int id) const
{
  prop.remove(getCameraKey("p1", id).c_str());
  prop.remove(getCameraKey("p2", id).c_str());

  prop.remove(getCameraKey("k1", id).c_str());
  prop.remove(getCameraKey("k2", id).c_str());
  prop.remove(getCameraKey("k3", id).c_str());
  prop.remove(getCameraKey("k4", id).c_str());
  prop.remove(getCameraKey("k5", id).c_str());
  prop.remove(getCameraKey("k6", id).c_str());

  prop.remove(getCameraKey("s1", id).c_str());
  prop.remove(getCameraKey("s2", id).c_str());
  prop.remove(getCameraKey("s3", id).c_str());
  prop.remove(getCameraKey("s4", id).c_str());
}

// --------------------- Equidistant ---------------------

EquidistantDistortion::EquidistantDistortion()
{
  for (int i=0; i<4; i++)
  {
    ed[i]=0;
  }
}

EquidistantDistortion::EquidistantDistortion(const gutil::Properties &prop, int id)
{
  prop.getValue(getCameraKey("e1", id).c_str(), ed[0], "0");
  prop.getValue(getCameraKey("e2", id).c_str(), ed[1], "0");
  prop.getValue(getCameraKey("e3", id).c_str(), ed[2], "0");
  prop.getValue(getCameraKey("e4", id).c_str(), ed[3], "0");
}

Distortion *EquidistantDistortion::clone() const
{
  EquidistantDistortion *ret=new EquidistantDistortion();

  for (int i=0; i<4; i++)
  {
    ret->ed[i]=ed[i];
  }

  return ret;
}

int EquidistantDistortion::countParameter() const
{
  return 4;
}

double EquidistantDistortion::getParameter(int i) const
{
  return ed[i];
}

void EquidistantDistortion::setParameter(int i, double v)
{
  ed[i]=v;
}

void EquidistantDistortion::transform(double &x, double &y, double xd, double yd) const
{
  const double radius=std::sqrt(xd*xd+yd*yd);

  if (radius > 1e-9)
  {
    const double theta=std::atan(radius);
    const double theta2=theta*theta;
    const double theta4=theta2*theta2;
    const double theta6=theta4*theta2;
    const double theta8=theta4*theta4;
    const double td=theta*(1+ed[0]*theta2+ed[1]*theta4+ed[2]*theta6+ed[3]*theta8);

    xd*=td/radius;
    yd*=td/radius;
  }

  x=xd;
  y=yd;
}

namespace
{

struct EquidistantDistortionParameter
{
  double ed[4];
  double radius;
};

int computeEquidistantDistortion(int n, double x[], int m, double fvec[], void *up)
{
  EquidistantDistortionParameter *p=static_cast<EquidistantDistortionParameter *>(up);

  const double theta=std::atan(x[0]);
  const double theta2=theta*theta;
  const double theta4=theta2*theta2;
  const double theta6=theta4*theta2;
  const double theta8=theta4*theta4;
  const double td=theta*(1+p->ed[0]*theta2+p->ed[1]*theta4+p->ed[2]*theta6+p->ed[3]*theta8);

  fvec[0]=p->radius-td;

  return 0;
}

}

void EquidistantDistortion::invTransform(double &xd, double &yd, double xx, double yy) const
{
  // equidistant distortion model

  double x[1]= { std::sqrt(xx*xx+yy*yy) };

  xd=xx;
  yd=yy;

  if (x[0] > 1e-9)
  {
    EquidistantDistortionParameter param;

    param.ed[0]=ed[0];
    param.ed[1]=ed[1];
    param.ed[2]=ed[2];
    param.ed[3]=ed[3];
    param.radius=x[0];

    double fvec[1];

    long ltmp[1]= {0};
    double dtmp[7];

    memset(dtmp, 0, 7*sizeof(double));
    gmath::slmdif(computeEquidistantDistortion, 1, 1, x, fvec, &param, 1e-6, 0, ltmp, dtmp);

    xd*=x[0]/param.radius;
    yd*=x[0]/param.radius;
  }
}

void EquidistantDistortion::getProperties(gutil::Properties &prop, int id) const
{
  prop.putValue(getCameraKey("e1", id).c_str(), ed[0]);
  prop.putValue(getCameraKey("e2", id).c_str(), ed[1]);
  prop.putValue(getCameraKey("e3", id).c_str(), ed[2]);
  prop.putValue(getCameraKey("e4", id).c_str(), ed[3]);
}

void EquidistantDistortion::cleanProperties(gutil::Properties &prop, int id) const
{
  prop.remove(getCameraKey("e1", id).c_str());
  prop.remove(getCameraKey("e2", id).c_str());
  prop.remove(getCameraKey("e3", id).c_str());
  prop.remove(getCameraKey("e4", id).c_str());
}

}
