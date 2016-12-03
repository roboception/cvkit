/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2016 Roboception GmbH
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

#include "estimatedplane.h"

namespace gmath
{

EstimatedPlane::EstimatedPlane()
{
  clear();
}

void EstimatedPlane::clear()
{
  sx=sy=sz=0;
  sxx=sxy=sxz=syy=syz=0;
  n=0;

  a=b=c=std::numeric_limits<double>::infinity();
}

void EstimatedPlane::add(double x, double y, double z)
{
  sx+=x;
  sy+=y;
  sz+=z;
  sxx+=x*x;
  sxy+=x*y;
  sxz+=x*z;
  syy+=y*y;
  syz+=y*z;
  n++;

  a=std::numeric_limits<double>::infinity();
}

double EstimatedPlane::getNormal(Vector3d N)
{
  if (!std::isfinite(a))
  {
    computePlane();
  }

  double div=sqrt(a*a+b*b+1);

  N[0]=-a/div;
  N[1]=-b/div;
  N[2]=1/div;

  return c/div;
}

void EstimatedPlane::computePlane()
{
  if (n >= 3)
  {
    double div=n*sxx*syy+2*sx*sy*sxy-syy*sx*sx-sxx*sy*sy-n*sxy*sxy;

    if (fabs(div) != 0)
    {
      a=(sxz*(n*syy-sy*sy)+syz*(sx*sy-n*sxy)+sz*(sxy*sy-sx*syy))/div;
      b=(sxz*(sx*sy-n*sxy)+syz*(n*sxx-sx*sx)+sz*(sx*sxy-sxx*sy))/div;
      c=(sxz*(sxy*sy-sx*syy)+syz*(sx*sxy-sxx*sy)+sz*(sxx*syy-sxy*sxy))/div;
    }
  }
}

}