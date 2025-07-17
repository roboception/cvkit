/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2025, Roboception GmbH
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

#include "homography.h"
#include "linalg.h"

#include <exception>

namespace gmath
{

Matrix33d getHomography(const std::vector<Vector2d> &q, const std::vector<Vector2d> &p)
{
  if (p.size() != q.size())
  {
    throw std::invalid_argument("Vectors given to getHomography() must have same size: "+
      std::to_string(q.size())+" != "+std::to_string(p.size()));
  }

  if (p.size() < 4)
  {
    throw std::invalid_argument("At least four point pairs must be given to getHomography(): "+
      std::to_string(p.size()));
  }

  Matrixd M(2*p.size(), 9);

  for (size_t i=0; i<p.size(); i++)
  {
    int k=2*i;
    M(k, 0)=p[i][0];
    M(k, 1)=p[i][1];
    M(k, 2)=1;
    M(k, 3)=0;
    M(k, 4)=0;
    M(k, 5)=0;
    M(k, 6)=-q[i][0]*p[i][0];
    M(k, 7)=-q[i][0]*p[i][1];
    M(k, 8)=-q[i][0];

    k++;
    M(k, 0)=0;
    M(k, 1)=0;
    M(k, 2)=0;
    M(k, 3)=p[i][0];
    M(k, 4)=p[i][1];
    M(k, 5)=1;
    M(k, 6)=-q[i][1]*p[i][0];
    M(k, 7)=-q[i][1]*p[i][1];
    M(k, 8)=-q[i][1];
  }

  Matrixd U, V;
  Vectord W;

  svd(M, U, W, V);

  // solution of the homogeneous linear equation system is the last column of V

  Vectord h=V.getColumn(V.cols()-1);
  Matrix33d H;

  H(0, 0)=h[0];
  H(0, 1)=h[1];
  H(0, 2)=h[2];
  H(1, 0)=h[3];
  H(1, 1)=h[4];
  H(1, 2)=h[5];
  H(2, 0)=h[6];
  H(2, 1)=h[7];
  H(2, 2)=h[8];

  return H;
}

}
