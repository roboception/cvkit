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

#ifndef GMATH_ESTIMATEDPLANE_H
#define GMATH_ESTIMATEDPLANE_H

#include <gmath/svector.h>
#include <limits>

namespace gmath
{

/**
  Plane that is estimated from a number of sample points.
*/

class EstimatedPlane
{
  public:

    EstimatedPlane();

    /**
      Clears the plane.
    */

    void clear();

    /**
      Returns if the plane is valid.

      @return True if the plane is valid.
    */

    bool isValid()
    {
      if (!std::isfinite(a))
      {
        computePlane();
      }

      return std::isfinite(a);
    }

    /**
      Adds a point to the plane. The plane is estimated such that the distance
      to all given points is minimal in z direction.

      @param x, y, z Coordinates of the plane.
    */

    void add(double x, double y, double z);

    /**
      Returns the normal vector and distance of the plane to the origin. The
      equation of the plane is N*p=d. The normal vector is 0 if the plane is
      undefined.

      @param N Vector that will be set to the plane normal on return
      @return  Distance of the plane from the origin.
    */

    double getNormal(Vector3d N);

    /**
      Returns the Z value of the plane at the coordinate x and y.

      @param x, y Position of point.
      @return z   Z value of the plane at the given position.
    */

    double getZ(double x, double y)
    {
      if (!std::isfinite(a))
      {
        computePlane();
      }

      return a*x+b*y+c;
    }

  private:

    void computePlane();

    double sx, sy, sz, sxx, sxy, sxz, syy, syz;
    double a, b, c;
    int n;
};

}

#endif
