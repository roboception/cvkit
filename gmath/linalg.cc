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

#include "linalg.h"

#include <cmath>

using std::abs;
using std::max;
using std::min;

namespace gmath
{

const double pi=3.14159265358979323846;

Matrix33d createRx(double a)
{
    Matrix33d ret;
    
    ret(0, 0)=1.0;
    ret(0, 1)=0.0;
    ret(0, 2)=0.0;
    
    ret(1, 0)=0.0;
    ret(1, 1)=cos(a);
    ret(1, 2)=-sin(a);
    
    ret(2, 0)=0.0;
    ret(2, 1)=sin(a);
    ret(2, 2)=cos(a);
    
    return ret;
}

Matrix33d createRy(double a)
{
    Matrix33d ret;
    
    ret(0, 0)=cos(a);
    ret(0, 1)=0.0;
    ret(0, 2)=sin(a);
    
    ret(1, 0)=0.0;
    ret(1, 1)=1.0;
    ret(1, 2)=0.0;
    
    ret(2, 0)=-sin(a);
    ret(2, 1)=0.0;
    ret(2, 2)=cos(a);
    
    return ret;
}

Matrix33d createRz(double a)
{
    Matrix33d ret;
    
    ret(0, 0)=cos(a);
    ret(0, 1)=-sin(a);
    ret(0, 2)=0.0;
    
    ret(1, 0)=sin(a);
    ret(1, 1)=cos(a);
    ret(1, 2)=0.0;
    
    ret(2, 0)=0.0;
    ret(2, 1)=0.0;
    ret(2, 2)=1.0;
    
    return ret;
}

Matrix33d createR(double ax, double ay, double az)
{
    Matrix33d ret;
    
    ret(0, 0)=cos(ay)*cos(az);
    ret(0, 1)=-cos(ay)*sin(az);
    ret(0, 2)=-sin(ay);

    ret(1, 0)=-sin(ax)*sin(ay)*cos(az)+cos(ax)*sin(az);
    ret(1, 1)=sin(ax)*sin(ay)*sin(az)+cos(ax)*cos(az);
    ret(1, 2)=-sin(ax)*cos(ay);

    ret(2, 0)=cos(ax)*sin(ay)*cos(az)+sin(ax)*sin(az);
    ret(2, 1)=-cos(ax)*sin(ay)*sin(az)+sin(ax)*cos(az);
    ret(2, 2)=cos(ax)*cos(ay);
    
    return ret;
}

bool recoverEuler(const Matrix33d &R, double &ax, double &ay, double &az, bool tolerant)
{
    int    i, k;
    double pos[8][3];
    double e, emin;
    
      // checking if cos(ay) is 0 or not
    
    if (abs(R(0, 2)) < 1.0-1e-10)
    {
        // cos(ay) is not 0
      
        // creating 8 possible sets of angles: two possibilities for ay
      
      pos[0][1]=pos[1][1]=pos[2][1]=pos[3][1]=asin(-R(0, 2));
      pos[4][1]=pos[5][1]=pos[6][1]=pos[7][1]=pi-pos[0][1];
      
        // for each possibility of ay there are two possibilities of ax
      
      pos[0][0]=pos[1][0]=asin(-R(1, 2)/cos(pos[0][1]));
      pos[2][0]=pos[3][0]=pi-pos[0][0];

      pos[4][0]=pos[5][0]=asin(-R(1, 2)/cos(pos[4][1]));
      pos[6][0]=pos[7][0]=pi-pos[4][0];

        // for each possibility of ay there are two possibilities of az as
        // well, which have to be combined with all possibilities of ax
      
      pos[0][2]=pos[2][2]=asin(-R(0, 1)/cos(pos[0][1]));
      pos[1][2]=pos[3][2]=pi-pos[0][2];

      pos[4][2]=pos[6][2]=asin(-R(0, 1)/cos(pos[4][1]));
      pos[5][2]=pos[7][2]=pi-pos[4][2];
      
        // check all 8 possible sets of angles with the remaining 6 equations
        // and use the set with the lowest error
      
      k=-1;
      emin=0;
      for (i=0; i<8; i++)
      {
        e=abs(R(0, 0)-cos(pos[i][1])*cos(pos[i][2]));
        
        e+=abs(R(2, 2)-cos(pos[i][0])*cos(pos[i][1]));
        
        e+=abs(R(1, 0)+sin(pos[i][0])*sin(pos[i][1])*
          cos(pos[i][2])-cos(pos[i][0])*sin(pos[i][2]));
                  
        e+=abs(R(1, 1)-sin(pos[i][0])*sin(pos[i][1])*
          sin(pos[i][2])-cos(pos[i][0])*cos(pos[i][2]));
        
        e+=abs(R(2, 0)-cos(pos[i][0])*sin(pos[i][1])*
          cos(pos[i][2])-sin(pos[i][0])*sin(pos[i][2]));
        
        e+=abs(R(2, 1)+cos(pos[i][0])*sin(pos[i][1])*
          sin(pos[i][2])-sin(pos[i][0])*cos(pos[i][2]));
        
        if (k < 0 || e < emin)
        {
          k=i;
          emin=e;
        }
      }

      if (!tolerant && emin > 1e-6)
        return false;

      ax=pos[k][0];
      ay=pos[k][1];
      az=pos[k][2];
    }
    else
    {
        // cos(ay) is 0
      
      if (R(0, 2) > 0.0)
      {
        ax=0.0;
        ay=-pi/2.0;
        az=asin(R(1, 0));
        
        if (abs(R(1, 1)-cos(az)) > 1e-6)
          az=pi-az;
      }
      else
      {
        ax=0.0;
        ay=pi/2.0;
        az=asin(R(1, 0));
        
        if (abs(R(1, 1)-cos(az)) > 1e-6)
          az=pi-az;
      }
    }
    
      // make sure that for all angles -PI < a <= PI
    
    while (ax <= pi)
      ax+=2.0*pi;

    while (ax > pi)
      ax-=2.0*pi;
    
    while (ay <= pi)
      ay+=2.0*pi;

    while (ay > pi)
      ay-=2.0*pi;

    while (az <= pi)
      az+=2.0*pi;

    while (az > pi)
      az-=2.0*pi;

    return true;
}

Matrix33d createR(const Vector3d &n, double phi)
{
    double    l;
    double    n1, n2, n3;
    Matrix33d R;
    
    l=norm(n);
    
    if (l > 0)
    {
      Matrix33d m1, m2, m3;
      
      n1=n[0]/l;
      n2=n[1]/l;
      n3=n[2]/l;
      
      m2(0, 0)=n1*n1; m2(0, 1)=n1*n2; m2(0, 2)=n1*n3;
      m2(1, 0)=n2*n1; m2(1, 1)=n2*n2; m2(1, 2)=n2*n3;
      m2(2, 0)=n3*n1; m2(2, 1)=n3*n2; m2(2, 2)=n3*n3;
      
      m3(0, 0)=  0; m3(0, 1)=-n3; m3(0, 2)= n2;
      m3(1, 0)= n3; m3(1, 1)=  0; m3(1, 2)=-n1;
      m3(2, 0)=-n2; m3(2, 1)= n1; m3(2, 2)=  0;
      
      R=cos(phi)*m1 + (1-cos(phi))*m2 + sin(phi)*m3;
    }
    
    return R;
}

double recoverAngleAxis(const Matrix33d &R, Vector3d &n)
{
    double v;
    double cos_phi, sin_phi;
    double phi;
    
      /*
        The three eigenvalues of a rotation matrix are:
        x1=1, x2=cos(phi) + i*sin(phi), x3=cos(phi) - i*sin(phi)
        
        The eigenvalues are the solutions of the characteristic polynom
        det(R - xI)=0.
        
        The rotation axis n is the eigenvector of R that corresponds to the
        eigenvalue 1.
        
        Solving this explicitely for a rotation matrix results in:
        
          (r21-r12)
        v=(r02-r20)
          (r10-r01)
        
        n=v/|v|
        
        cos(phi)=(r11+r22+r33-1)/2
        sin(phi)=|v|/2
        phi = atan(sin(phi)/cos(phi))
        
        (see also book of Trucco and Verri, 1998).
      */
    
    cos_phi=(R(0, 0)+R(1, 1)+R(2, 2)-1.0)/2;
    
    if (cos_phi > 1-1e-9) /* special case: 0 degree */
    {
      phi=0;
      n[0]=1;
      n[1]=0;
      n[2]=0;
    }
    else if (cos_phi < -1+1e-9) /* special case: 180 degree */
    {
      phi=pi;
      if (R(0, 0) >= R(1, 1))
      {
        if (R(0, 0) >= R(2, 2))
        {
          n[0]=sqrt(R(0, 0)-R(1, 1)-R(2, 2)+1)/2;
          v=1/(2*n[0]);
          n[1]=v*R(0, 1);
          n[2]=v*R(0, 2);
        }
        else
        {
          n[2]=sqrt(R(2, 2)-R(0, 0)-R(1, 1)+1)/2;
          v=1/(2*n[2]);
          n[0]=v*R(0, 2);
          n[1]=v*R(1, 2);
        }
      }
      else
      {
        if (R(1, 1) >= R(2, 2))
        {
          n[1]=sqrt(R(1, 1)-R(0, 0)-R(2, 2)+1)/2;
          v=1/(2*n[1]);
          n[0]=v*R(0, 1);
          n[2]=v*R(1, 2);
        }
        else
        {
          n[2]=sqrt(R(2, 2)-R(0, 0)-R(1, 1)+1)/2;
          v=1/(2*n[2]);
          n[0]=v*R(0, 2);
          n[1]=v*R(1, 2);
        }
      }
    }
    else /* general case */
    {
      n[0]=R(2, 1)-R(1, 2);
      n[1]=R(0, 2)-R(2, 0);
      n[2]=R(1, 0)-R(0, 1);
      
      v=norm(n);
      
      n/=v;
      sin_phi=v/2;
      
      phi=atan2(sin_phi, cos_phi);
    }
    
    return phi;
}

Matrix33d ensureRotation(const Matrix33d &R)
{
    Vector3d n;
    double phi=recoverAngleAxis(R, n);
    return createR(n, phi);
}

double transformGaussJordan(Matrixd &a)
{
    double ret=1;
    
      // compute a meaningful epsilon
    
    double eps=0;
    for (int k=0; k<a.rows(); k++)
    {
      double v=0;
      for (int i=0; i<a.cols(); i++)
        v+=abs(a(k, i));
      
      eps=max(eps, v);
    }
    
    int n=max(a.rows(), a.cols());
    
    eps*=n;
    eps*=pow(2, -52);
    
      // for all rows
    
    n=min(a.rows(), a.cols());
    for (int j=0; j<n; j++)
    {
        // find pivot element
      
      int pivot=j;
      for (int k=j+1; k<a.rows(); k++)
      {
        if (abs(a(k, j)) > abs(a(pivot, j)))
          pivot=k;
      }
      
        // stop, if pivot element is 0
      
      if (abs(a(pivot, j)) < eps)
      {
        ret=0;
        break;
      }
      
        // interchange line j with line with pivot element
      
      if (pivot != j)
      {
        for (int i=j; i<a.cols(); i++)
        {
          double v=a(j, i);
          a(j, i)=a(pivot, i);
          a(pivot, i)=v;
        }
        
        ret*=-1;
      }
      
      pivot=j;
      
        // scale pivot line such that the pivot element becomes 1
      
      double s=a(pivot, pivot);
      for (int i=pivot; i<a.cols(); i++)
        a(pivot, i)/=s;
      
      ret*=s;
      
        // eliminate all elements of the pivot column except the pivot
        // element
      
      for (int k=0; k<a.rows(); k++)
      {
        if (k != pivot)
        {
          double v=a(k, pivot);
          for (int i=pivot; i<a.cols(); i++)
            a(k, i)-=v*a(pivot, i);
        }
      }
    }
    
    return ret;
}

double det(const Matrixd &a)
{
    assert(a.rows() == a.cols());
    
    Matrixd b(a);
    return transformGaussJordan(b);
}

Matrix33d inv(const Matrix33d &a)
{
    double d=det(a);
    
    assert(d != 0);
    
    d=1.0/d;
    
    Matrix33d ret;
    
    ret(0, 0)=d*(a(1, 1)*a(2, 2)-a(1, 2)*a(2, 1));
    ret(0, 1)=d*(a(0, 2)*a(2, 1)-a(0, 1)*a(2, 2));
    ret(0, 2)=d*(a(0, 1)*a(1, 2)-a(0, 2)*a(1, 1));
    
    ret(1, 0)=d*(a(1, 2)*a(2, 0)-a(1, 0)*a(2, 2));
    ret(1, 1)=d*(a(0, 0)*a(2, 2)-a(0, 2)*a(2, 0));
    ret(1, 2)=d*(a(0, 2)*a(1, 0)-a(0, 0)*a(1, 2));
    
    ret(2, 0)=d*(a(1, 0)*a(2, 1)-a(1, 1)*a(2, 0));
    ret(2, 1)=d*(a(0, 1)*a(2, 0)-a(0, 0)*a(2, 1));
    ret(2, 2)=d*(a(0, 0)*a(1, 1)-a(0, 1)*a(1, 0));
    
    return ret;
}

namespace
{

double hypot(const double &a, const double &b)
{
    if (a == 0)
      return abs(b);
    
    double c=b/a;
    return abs(a)*sqrt(1+c*c);
}

}

// The SVD implementation is an adaptation from JAMA, a Java Matrix Library.
// See http://math.nist.gov/javanumerics/jama
//
// The copyright notice says: This software is a cooperative product of The
// MathWorks and the National Institute of Standards and Technology (NIST)
// which has been released to the public domain. Neither The MathWorks nor
// NIST assumes any responsibility whatsoever for its use by other parties,
// and makes no guarantees, expressed or implied, about its quality,
// reliability, or any other characteristic.
//
// The webpage also says: As Jama is in the public domain other developers are
// free to adopt and adapt this code to other styles of programming or to
// extend or modernize the API

void svd(const Matrixd &a, Matrixd &U, Vectord &W, Matrixd &V, bool thin)
{
    const bool wantu=true;
    const bool wantv=true;
    
    Matrixd A(a);
    
    const int m=A.rows();
    const int n=A.cols();
    const int ncu=thin?min(m, n):m;
    
    U.init(m, ncu); U=0;
    W.init(min(m+1, n));
    V.init(n, n); V=0;
    
    Vectord e(n);
    Vectord work(m);
    
    // Reduce A to bidiagonal form, storing the diagonal elements
    // in W and the super-diagonal elements in e.
    
    int nct=min(m-1, n);
    int nrt=max(0, min(n-2, m));
    int lu=max(nct, nrt);
    
    for (int k=0; k<lu; k++)
    {
      if (k < nct)
      {
        // Compute the transformation for the k-th column and
        // place the k-th diagonal in W[k].
        // Compute 2-norm of k-th column without under/overflow.
        
        W[k]=0;
        for (int i=k; i<m; i++)
          W[k]=hypot(W[k], A(i, k));
          
        if (W[k] != 0.0)
        {
          if (A(k, k) < 0.0)
            W[k]=-W[k];
            
          for (int i=k; i<m; i++)
            A(i, k)/=W[k];
            
          A(k, k)+=1.0;
        }
        
        W[k]=-W[k];
      }
      
      for (int j=k+1; j<n; j++)
      {
        if ((k < nct) & (W[k] != 0.0))
        {
          // Apply the transformation.
          
          double t=0;
          for (int i=k; i<m; i++)
            t+=A(i, k)*A(i, j);
            
          t=-t/A(k, k);
          for (int i=k; i<m; i++)
            A(i, j)+=t*A(i, k);
        }
        
        // Place the k-th row of A into e for the
        // subsequent calculation of the row transformation.
        
        e[j]=A(k, j);
      }
      
      if (wantu & (k < nct))
      {
        // Place the transformation in U for subsequent back
        // multiplication.
        
        for (int i=k; i<m; i++)
          U(i, k)=A(i, k);
      }
      
      if (k < nrt)
      {
        // Compute the k-th row transformation and place the
        // k-th super-diagonal in e[k].
        // Compute 2-norm without under/overflow.
        
        e[k]=0;
        for (int i=k+1; i<n; i++)
          e[k]=hypot(e[k], e[i]);
          
        if (e[k] != 0.0)
        {
          if (e[k+1] < 0.0)
            e[k]=-e[k];
            
          for (int i=k+1; i<n; i++)
            e[i]/=e[k];
            
          e[k+1]+=1.0;
        }
        
        e[k]=-e[k];
        if ((k+1 < m) & (e[k] != 0.0))
        {
          // Apply the transformation.
          
          for (int i=k+1; i<m; i++)
            work[i]=0.0;
            
          for (int j=k+1; j<n; j++)
          {
            for (int i=k+1; i<m; i++)
              work[i]+=e[j]*A(i, j);
          }
          
          for (int j=k+1; j<n; j++)
          {
            double t=-e[j]/e[k+1];
            
            for (int i=k+1; i<m; i++)
              A(i, j)+=t*work[i];
          }
        }
        
        if (wantv)
        {
          // Place the transformation in V for subsequent
          // back multiplication.
          
          for (int i=k+1; i<n; i++)
            V(i, k)=e[i];
        }
      }
    }
    
    // Set up the final bidiagonal matrix or order p.
    
    int p=min(n, m+1);
    
    if (nct < n)
      W[nct]=A(nct, nct);
      
    if (m < p)
      W[p-1]=0.0;
      
    if (nrt+1 < p)
      e[nrt]=A(nrt, p-1);
      
    e[p-1]=0.0;
    
    // If required, generate U.
    
    if (wantu)
    {
      for (int j=nct; j<ncu; j++)
      {
        for (int i=0; i<m; i++)
          U(i, j)=0.0;
          
        U(j, j)=1.0;
      }
      
      for (int k=nct-1; k>=0; k--)
      {
        if (W[k] != 0.0)
        {
          for (int j=k+1; j<ncu; j++)
          {
            double t=0;
            
            for (int i=k; i<m; i++)
              t+=U(i, k)*U(i, j);
              
            t=-t/U(k, k);
            
            for (int i=k; i<m; i++)
              U(i, j)+=t*U(i, k);
          }
          
          for (int i=k; i<m; i++)
            U(i, k)=-U(i, k);
            
          U(k, k)+=1.0;
          
          for (int i=0; i<k-1; i++)
            U(i, k)=0.0;
        }
        else
        {
          for (int i=0; i<m; i++)
            U(i, k)=0.0;
            
          U(k, k)=1.0;
        }
      }
    }
    
    // If required, generate V.
    
    if (wantv)
    {
      for (int k=n-1; k>=0; k--)
      {
        if ((k < nrt) & (e[k] != 0.0))
        {
          for (int j=k+1; j<n; j++)
          {
            double t=0;
            
            for (int i=k+1; i<n; i++)
              t+=V(i, k)*V(i, j);
              
            t=-t/V(k+1, k);
            
            for (int i=k+1; i<n; i++)
              V(i, j)+=t*V(i, k);
          }
        }
        
        for (int i=0; i<n; i++)
          V(i, k)=0.0;
          
        V(k, k)=1.0;
      }
    }
    
    // Main iteration loop for the singular values.
    
    int pp=p-1;
    int iter=0;
    double eps=pow(2.0,-52.0);
    double tiny=pow(2.0,-966.0);
    
    while (p > 0)
    {
      int k, kase;
      
      // Here is where a test for too many iterations would go.
      
      // This section of the program inspects for
      // negligible elements in the s and e arrays. On
      // completion the variables kase and k are set as follows.
      
      // kase = 1     if s(p) and e[k-1] are negligible and k<p
      // kase = 2     if s(k) is negligible and k<p
      // kase = 3     if e[k-1] is negligible, k<p, and
      //              s(k), ..., s(p) are not negligible (qr step).
      // kase = 4     if e(p-1) is negligible (convergence).
      
      for (k=p-2; k>=-1; k--)
      {
        if (k == -1)
          break;
          
        if (abs(e[k]) <= tiny+eps*(abs(W[k])+abs(W[k+1])))
        {
          e[k]=0.0;
          break;
        }
      }
      
      if (k == p-2)
      {
        kase=4;
      }
      else
      {
        int ks;
        
        for (ks=p-1; ks>=k; ks--)
        {
          if (ks == k)
            break;
            
          double t=(ks != p ? abs(e[ks]) : 0.) +
                   (ks != k+1 ? abs(e[ks-1]) : 0.);
                     
          if (abs(W[ks]) <= tiny+eps*t)
          {
            W[ks]=0.0;
            break;
          }
        }
        
        if (ks == k)
        {
          kase=3;
        }
        else if (ks == p-1)
        {
          kase=1;
        }
        else
        {
          kase=2;
          k=ks;
        }
      }
      
      k++;
      
      // Perform the task indicated by kase.
      
      switch (kase)
      {
      
        // Deflate negligible s(p).
        
        case 1:
          {
            double f=e[p-2];
            e[p-2]=0.0;
            
            for (int j=p-2; j>=k; j--)
            {
              double t=hypot(W[j],f);
              double cs=W[j]/t;
              double sn=f/t;
              W[j]=t;
              
              if (j != k)
              {
                f=-sn*e[j-1];
                e[j-1]=cs*e[j-1];
              }
              
              if (wantv)
              {
                for (int i=0; i<n; i++)
                {
                  t=cs*V(i, j)+sn*V(i, p-1);
                  V(i, p-1)=-sn*V(i, j)+cs*V(i, p-1);
                  V(i, j)=t;
                }
              }
            }
          }
          break;
          
        // Split at negligible s(k).
        
        case 2:
          {
            double f=e[k-1];
            e[k-1]=0.0;
            
            for (int j=k; j<p; j++)
            {
              double t=hypot(W[j],f);
              double cs=W[j]/t;
              double sn=f/t;
              W[j]=t;
              f=-sn*e[j];
              e[j]=cs*e[j];
              
              if (wantu)
              {
                for (int i=0; i<m; i++)
                {
                  t=cs*U(i, j)+sn*U(i, k-1);
                  U(i, k-1)=-sn*U(i, j)+cs*U(i, k-1);
                  U(i, j)=t;
                }
              }
            }
          }
          break;
          
        // Perform one qr step.
        
        case 3:
          {
          
            // Calculate the shift.
            
            double scale=max(max(max(max(abs(W[p-1]), abs(W[p-2])), abs(e[p-2])),
                         abs(W[k])), abs(e[k]));
            double sp=W[p-1]/scale;
            double spm1=W[p-2]/scale;
            double epm1=e[p-2]/scale;
            double sk=W[k]/scale;
            double ek=e[k]/scale;
            double b=((spm1+sp)*(spm1-sp)+epm1*epm1)/2.0;
            double c=(sp*epm1)*(sp*epm1);
            double shift=0.0;
            
            if ((b != 0.0) | (c != 0.0))
            {
              shift=sqrt(b*b + c);
              
              if (b < 0.0)
                shift=-shift;
                
              shift=c/(b + shift);
            }
            
            double f=(sk + sp)*(sk - sp) + shift;
            double g=sk*ek;
            
            // Chase zeros.
            
            for (int j=k; j<p-1; j++)
            {
              double t=hypot(f, g);
              double cs=f/t;
              double sn=g/t;
              
              if (j != k)
                e[j-1]=t;
                
              f=cs*W[j]+sn*e[j];
              e[j]=cs*e[j]-sn*W[j];
              g=sn*W[j+1];
              W[j+1]=cs*W[j+1];
              
              if (wantv)
              {
                for (int i=0; i<n; i++)
                {
                  t=cs*V(i, j)+sn*V(i, j+1);
                  V(i, j+1)=-sn*V(i, j)+cs*V(i, j+1);
                  V(i, j)=t;
                }
              }
              
              t=hypot(f,g);
              cs=f/t;
              sn=g/t;
              W[j]=t;
              f=cs*e[j]+sn*W[j+1];
              W[j+1]=-sn*e[j]+cs*W[j+1];
              g=sn*e[j+1];
              e[j+1]=cs*e[j+1];
              
              if (wantu && (j < m-1))
              {
                for (int i=0; i<m; i++)
                {
                  t=cs*U(i, j)+sn*U(i, j+1);
                  U(i, j+1)=-sn*U(i, j)+cs*U(i, j+1);
                  U(i, j)=t;
                }
              }
            }
            
            e[p-2]=f;
            iter++;
          }
          break;
          
        // Convergence.
        
        case 4:
          {
          
            // Make the singular values positive.
            
            if (W[k] <= 0.0)
            {
              W[k]=(W[k] < 0.0 ? -W[k] : 0.0);
              
              if (wantv)
              {
                for (int i=0; i<n; i++)
                  V(i, k)=-V(i, k);
              }
            }
            
            // Order the singular values.
            
            while (k < pp)
            {
              if (W[k] >= W[k+1])
                break;
                
              double t=W[k];
              W[k]=W[k+1];
              W[k+1]=t;
              
              if (wantv && (k < n-1))
              {
                for (int i=0; i<n; i++)
                {
                  t=V(i, k+1);
                  V(i, k+1)=V(i, k);
                  V(i, k)=t;
                }
              }
              
              if (wantu && (k < m-1))
              {
                for (int i=0; i<m; i++)
                {
                  t=U(i, k+1);
                  U(i, k+1)=U(i, k);
                  U(i, k)=t;
                }
              }
              
              k++;
            }
            
            iter=0;
            p--;
          }
          break;
      }
    }
}

}
