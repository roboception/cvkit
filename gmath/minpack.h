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

#ifndef GMATH_MINPACK_H
#define GMATH_MINPACK_H

namespace gmath
{

/**
  Computes the root-mean-square of the values of the given vector.
  
  @param m    Number of functions.
  @param fvec Returned list of function values of size m.
  @return     Root mean square of fvec.
*/

double rms(int m, double fvec[]);

/**
  Prototype for calculating the function values fvec at x, i.e.
  
  fvec[0] = f_0(x[0], ... x[n])
  fvec[m] = f_m(x[0], ... x[n])
  
  @param n    Number of variables.
  @param x    List of variables of size n.
  @param m    Number of functions.
  @param fvec Returned list of function values of size m.
  @param up   User pointer that was passed to lmdif().
  @return     >= 0 for continuing the optimization, < 0 to abort.
*/

typedef int (*lmdifFct) (int n, double x[], int m, double fvec[], void *up);

/**
  Prototype for calculating the function fvec or jacobian fjac at x. fvec and
  fjac may be NULL, if calculation is not required.
  
  fvec[0] = f_0(x[0], ... x[n])
  fvec[m] = f_m(x[0], ... x[n])
  
  @param n    Number of variables.
  @param x    List of variables of size n.
  @param m    Number of functions.
  @param fvec Returned list of function values of size m. May be 0.
  @param fjac Returned jacobian of size m*n. May be 0.
  @param up   User pointer that was passed to lmder().
  @return     >= 0 for continuing the optimization, < 0 to abort.
*/

typedef int (*lmderFct) (int n, double x[], int m, double fvec[],
  double fjac[], void *up);

/**
  Short version of lmdif(), i.e. calls lmdif with some default parameters.
  
  @param fct  Function that calculates the function values for given x.
  @param m    Number of functions.
  @param n    Number of variables.
  @param x    List of variables of size n, with good initialised values.
  @param fvec List of function values of size m.
  @param up   Pointer to user data, which is passed to fct.
  @param tol  Threshold for aborting the optimization.
  @param ltmp Temporary array of size n.
  @param dtmp Temporary array of size 5*n+m*n+m.
  @return False, if the optminization failed.
*/

bool slmdif(lmdifFct fct, int m, int n, double x[], double fvec[],
  void *up=0, double tol=1e-12, long ltmp[]=0, double dtmp[]=0);

/**
  Short version of lmder(), i.e. calls lmder with some default parameters.
  
  @param fct  Function that calculates the function values or the jacobian for
              given x.
  @param m    Number of functions.
  @param n    Number of variables.
  @param x    List of variables of size n, with good initialised values.
  @param fvec List of function values of size m.
  @param up   Pointer to user data, which is passed to fct.
  @param tol  Threshold for aborting the optimization, e.g. 1e-12.
  @param ltmp Temporary array of size n.
  @param dtmp Temporary array of size 5*n+m*n+m.
  @return False, if the optminization failed.
*/

bool slmder(lmderFct fct, int m, int n, double x[], double fvec[],
  void *up=0, double tol=1e-12, long ltmp[]=0, double dtmp[]=0);

}

#endif
