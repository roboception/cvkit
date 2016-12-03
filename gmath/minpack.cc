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

#include <stdlib.h>
#include <string.h>

#include "minpack.h"
#include "minpack/f2c.h"

#include <assert.h>
#include <cmath>

#include <iostream>

/* The functions are copied as prototypes, so that the compiler can check
   if I made mistakes while calling functions with so many parameter ;-) */

extern "C"
{
int lmdif_cvkit(S_fp fct, integer *m, integer *n, doublereal *x,
        doublereal *fvec, doublereal *ftol, doublereal *xtol, doublereal *
        gtol, integer *maxfev, doublereal *epsfct, doublereal *diag, integer *
        mode, doublereal *factor, integer *nprint, integer *info, integer *
        nfev, doublereal *fjac, integer *ldfjac, integer *ipvt, doublereal *
        qtf, doublereal *wa1, doublereal *wa2, doublereal *wa3, doublereal *
        wa4, void *up);

int lmder_cvkit(S_fp fct, integer *m, integer *n, doublereal *x,
        doublereal *fvec, doublereal *fjac, integer *ldfjac, doublereal *ftol,
        doublereal *xtol, doublereal *gtol, integer *maxfev, doublereal *
        diag, integer *mode, doublereal *factor, integer *nprint, integer *
        info, integer *nfev, integer *njev, integer *ipvt, doublereal *qtf,
        doublereal *wa1, doublereal *wa2, doublereal *wa3, doublereal *wa4,
        void *up);
}

namespace gmath
{

double rms(int m, double fvec[])
{
  double v=0;

  for (int i=0; i<m; i++)
    v+=fvec[i]*fvec[i];

  return std::sqrt(v/m);
}

bool slmdif(lmdifFct fct, int m, int n, double x[], double fvec[], void *up,
  double tol, double step, long ltmp[], double dtmp[])
{
    integer    mp=static_cast<integer>(m);
    integer    np=static_cast<integer>(n);
    doublereal ftol=tol, xtol=tol, gtol=0;
    integer    maxfevl=static_cast<integer>(200*(n+1));
    doublereal epsfct=step;
    integer    model=1;
    doublereal factor=100;
    integer    nprint=-1;
    integer    info=0;
    integer    nfevl=0;
    integer    ldfjacl=(integer) m;

    long       *altmp=0;
    double     *adtmp=0;

    if (ltmp == 0)
      ltmp=altmp=static_cast<long *>(calloc(n, sizeof(long)));

    if (dtmp == 0)
      dtmp=adtmp=static_cast<double *>(calloc(5*n+m*n+m, sizeof(double)));

    lmdif_cvkit(reinterpret_cast<S_fp>(fct), &mp, &np, x, fvec, &ftol, &xtol, &gtol, &maxfevl, &epsfct,
      dtmp, &model, &factor, &nprint, &info, &nfevl, dtmp+n, &ldfjacl, ltmp,
      dtmp+n+m*n, dtmp+2*n+n*m, dtmp+3*n+n*m, dtmp+4*n+n*m, dtmp+5*n+n*m, up);

    free(altmp);
    free(adtmp);

    assert(info >= 1);

    return info <= 4;
}

bool slmder(lmderFct fct, int m, int n, double x[], double fvec[], void *up,
  double tol, long ltmp[], double dtmp[])
{
    integer    mp=static_cast<integer>(m);
    integer    np=static_cast<integer>(n);
    doublereal ftol=tol, xtol=tol, gtol=0;
    integer    maxfevl=static_cast<integer>(200*(n+1));
    integer    model=1;
    doublereal factor=100;
    integer    nprint=-1;
    integer    info=0;
    integer    nfevl=0, njevl=0;
    integer    ldfjacl=(integer) m;

    long       *altmp=0;
    double     *adtmp=0;

    if (ltmp == 0)
      ltmp=altmp=static_cast<long *>(calloc(n, sizeof(long)));

    if (dtmp == NULL)
      dtmp=adtmp=static_cast<double *>(calloc(5*n+m, sizeof(double)));

    lmder_cvkit(reinterpret_cast<S_fp>(fct), &mp, &np, x, fvec, dtmp+n, &ldfjacl, &ftol, &xtol, &gtol,
      &maxfevl, dtmp, &model, &factor, &nprint, &info, &nfevl, &njevl, ltmp,
      dtmp+n+m*n, dtmp+2*n+m*n, dtmp+3*n+m*n, dtmp+4*n+m*n, dtmp+5*n+m*n, up);

    free(altmp);
    free(adtmp);

    assert(info >= 1);

    return info <= 4;
}

}