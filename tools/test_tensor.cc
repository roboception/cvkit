/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2026 Roboception GmbH
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

#include "gmath/tensor.h"

#include <iostream>
#include <sstream>
#include <cmath>

bool check(const char *comment, const gmath::Tensord &T, const char *expected)
{
  std::ostringstream out;
  out << T;

  if (out.str() != expected)
  {
    std::cout << "Unexpected result: " << comment << std::endl;
    std::cout << "    " << out.str() << std::endl;
    std::cout << " != " << expected << std::endl;

    return false;
  }

  return true;
}

int main(int argc, const char *argv[])
{
  bool ret=true;

  try
  {
    std::cout << "--- Testing construction and reinitialization ---" << std::endl;

    // systematically testing all methods of Tensor

    ret=check("Constructor rank 0", gmath::Tensord(1), "{}[1]") && ret;
    ret=check("Constructor rank 1", gmath::Tensord(gmath::Vectord(1, 2, 3)), "{3}[1 2 3]") && ret;
    ret=check("Constructor rank 2", gmath::Tensord(gmath::Matrixd(3, 3)), "{3 3}[1 0 0 0 1 0 0 0 1]") && ret;

    {
      gmath::Tensord T(gmath::Vectord(1, 2, 3));
      gmath::Tensord S(T);

      ret=check("Copy constructor", S, "{3}[1 2 3]") && ret;
    }

    {
      gmath::Tensord T(gmath::Vectord(1, 2, 3));
      gmath::Tensord S(std::move(T));

      ret=check("Move constructor target", S, "{3}[1 2 3]") && ret;
    }

    ret=check("Shape constructor 1", gmath::Tensord({4}), "{4}[0 0 0 0]") && ret;
    ret=check("Shape constructor 2", gmath::Tensord({1, 2}), "{1 2}[0 0]") && ret;

    {
      gmath::Tensord T;

      T.init({1, 2, 1, 1});
      ret=check("init method", T, "{1 2 1 1}[0 0]") && ret;
    }



    std::cout << "--- Testing memory layout ---" << std::endl;

    {
      gmath::Matrixd M(2, 3);
      M(1, 2)=4;

      gmath::Tensord T(M);
      if (!T.isContiguous())
      {
        std::cout << "Unexpected result: Contiguous " << T.isContiguous() << " != 1" << std::endl;
        ret=false;
      }

      T.makeContiguous();
      ret=check("Make contiguous 1", T, "{2 3}[1 0 0 0 1 4]") && ret;

      T.setTransposed({1, 0}); // forcing to make it not contiguous
      T.makeContiguous();
      ret=check("Make contiguous 2", T, "{3 2}[1 0 0 1 0 4]") && ret;
    }



    std::cout << "--- Testing size and shape methods ---" << std::endl;

    {
      gmath::Matrixd M(2, 3);
      M(1, 2)=4;

      gmath::Tensord T(M);
      if (T.rank() != 2)
      {
        std::cout << "Unexpected result: Rank " << T.rank() << " != 2" << std::endl;
        ret=false;
      }

      if (T.shape(0) != 2 || T.shape(1) != 3)
      {
        std::cout << "Unexpected result: Shape {" << T.shape(0) << " " << T.shape(1) << "} != {2 3}" << std::endl;
        ret=false;
      }

      if (T.step()[0] != 3 || T.step()[1] != 1)
      {
        std::cout << "Unexpected result: Step {" << T.step()[0] << " " << T.step()[1] << "} != {3 1}" << std::endl;
        ret=false;
      }
    }



    std::cout << "--- Testing modifying shape and transpose indices ---" << std::endl;

    {
      {
        gmath::Tensord T(gmath::Vectord({3}));

        ret=check("Reduce rank 1", T, "{1}[3]") && ret;

        T.reduceRank();
        ret=check("Reduce rank 2", T, "{}[3]") && ret;

        T.init({1, 2, 1, 1});
        ret=check("Reduce rank 3", T, "{1 2 1 1}[0 0]") && ret;

        T.reduceRank();
        ret=check("Reduce rank 4", T, "{2}[0 0]") && ret;
      }

      gmath::Tensord T1(gmath::Vectord({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24}));
      gmath::Tensord T2(T1);

      T1.setReshaped({4, 2, 3});
      ret=check("Reshape 1", T1, "{4 2 3}[1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24]") && ret;

      T2.setReshaped({4, -1, 3});
      ret=check("Reshape 2", T2, "{4 2 3}[1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24]") && ret;

      T2.setTransposed({2, 1, -3});
      ret=check("Transpose", T2, "{3 2 4}[1 7 13 19 4 10 16 22 2 8 14 20 5 11 17 23 3 9 15 21 6 12 18 24]") && ret;

      T2.setReshaped({-1});
      ret=check("Reshape 3", T2, "{24}[1 7 13 19 4 10 16 22 2 8 14 20 5 11 17 23 3 9 15 21 6 12 18 24]") && ret;
    }



    std::cout << "--- Testing copy part ---" << std::endl;

    {
      gmath::Tensord T1(gmath::Vectord({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24}));
      T1.setReshaped({4, 3, 2});

      gmath::Tensord T2=T1.slice({gmath::TValue(1), gmath::TRange(1, 2, 1), gmath::TAll()});
      ret=check("Slice 1", T2, "{1 2 2}[9 10 11 12]") && ret;

      T2.reduceRank();
      ret=check("Reduce Rank 1", T2, "{2 2}[9 10 11 12]") && ret;

      gmath::Tensord T3=T1.slice({gmath::TRange(0, 4, 2), gmath::TValue(1), gmath::TValue(0)});
      ret=check("Slice 2", T3, "{2 1 1}[3 15]") && ret;

      T3.reduceRank();
      ret=check("Reduce Rank 2", T3, "{2}[3 15]") && ret;

      gmath::Tensord T4=T1.slice({gmath::TRange(2, 1, 2), gmath::TValue(1), gmath::TValue(0)});
      ret=check("Slice 3", T4, "{}[0]") && ret;
    }



    std::cout << "--- Testing copy and move operators ---" << std::endl;

    {
      gmath::Tensord T;

      T=3;
      ret=check("Copy from value", T, "{}[3]") && ret;

      T=gmath::Vectord(1, 2, 3);
      ret=check("Copy from vector", T, "{3}[1 2 3]") && ret;

      T=gmath::Matrixd(3, 3);
      ret=check("Copy from matrix", T, "{3 3}[1 0 0 0 1 0 0 0 1]") && ret;

      gmath::Tensord T2;
      T2=T;
      ret=check("Copy", T2, "{3 3}[1 0 0 0 1 0 0 0 1]") && ret;

      gmath::Tensord T3;
      T3=std::move(T2);

      ret=check("Move operator target", T3, "{3 3}[1 0 0 0 1 0 0 0 1]") && ret;
    }



    std::cout << "--- Testing cast operators ---" << std::endl;

    {
      gmath::Vectord V(1.2, 1.5, 4.2, 4.5);
      if (V != static_cast<gmath::Vectord>(gmath::Tensord(V)))
      {
        std::cout << "Unexpected result: Casting to vector" << std::endl;
        ret=false;
      }

      gmath::Matrixd M(4, 5);
      if (M != static_cast<gmath::Matrixd>(gmath::Tensord(M)))
      {
        std::cout << "Unexpected result: Casting to matrix" << std::endl;
        ret=false;
      }
    }



    std::cout << "--- Testing comparison operators ---" << std::endl;

    {
      gmath::Tensord T1(gmath::Vectord({1, 2, 3, 4, 5, 6}));
      T1.setReshaped({2, 3});

      gmath::Tensord T2(gmath::Vectord({1, 4, 2, 5, 3, 6}));
      T2.setReshaped({3, 2});
      T2.setTransposed({1, 0});

      gmath::Tensord T3(gmath::Vectord({7, 8, 9, 6, 5, 4}));
      T3.setReshaped({2, 3});

      if (!(T1 == T2))
      {
        std::cout << "Unexpected result: Operator ==" << std::endl;
        std::cout << T1 << std::endl;
        std::cout << T2 << std::endl;
        ret=false;
      }

      if (!(T1 != T3))
      {
        std::cout << "Unexpected result: Operator !=" << std::endl;
        std::cout << T1 << std::endl;
        std::cout << T3 << std::endl;
        ret=false;
      }

      T3.init({2, 3, 5});
      if (!(T1 != T3))
      {
        std::cout << "Unexpected result: Operator !=" << std::endl;
        std::cout << T1 << std::endl;
        std::cout << T3 << std::endl;
        ret=false;
      }
    }



    std::cout << "--- Testing element access ---" << std::endl;

    {
      gmath::Matrixd M(2, 3);
      M(0, 1)=2;
      M(1, 1)=3;
      M(1, 2)=4;

      gmath::Tensord T(M);
      if (T({0, 0}) != 1)
      {
        std::cout << "Unexpected result: Element access: " << T({0, 0}) << " != 1" << std::endl;
        ret=false;
      }

      if (T({1, 2}) != 4)
      {
        std::cout << "Unexpected result: Element access: " << T({1, 2}) << " != 4" << std::endl;
        ret=false;
      }

      if (T.at(4) != 3)
      {
        std::cout << "Unexpected result: Element access: " << T.at(4) << " != 3" << std::endl;
        ret=false;
      }

      T.setTransposed({-1, -2});

      if (T({2, 1}) != 4)
      {
        std::cout << "Unexpected result: Element access: " << T({2, 1}) << " != 4" << std::endl;
        ret=false;
      }
    }



    std::cout << "--- Testing arithmetic operators ---" << std::endl;

    {
      gmath::Tensord T1(gmath::Vectord({1, 2, 3, 4, 5, 6}));
      T1.setReshaped({2, 3});

      ret=check("Operator +", T1+T1, "{2 3}[2 4 6 8 10 12]") && ret;
      ret=check("Operator -", T1-T1, "{2 3}[0 0 0 0 0 0]") && ret;
      ret=check("Operator *", T1*T1, "{2 3}[1 4 9 16 25 36]") && ret;
      ret=check("Operator /", T1/T1, "{2 3}[1 1 1 1 1 1]") && ret;

      gmath::Tensord T2(gmath::Vectord({3, 4}));
      T2.setReshaped({2, 1});

      ret=check("Operator +", T1+T2, "{2 3}[4 5 6 8 9 10]") && ret;
      ret=check("Operator -", T1-T2, "{2 3}[-2 -1 0 0 1 2]") && ret;
      ret=check("Operator *", T1*T2, "{2 3}[3 6 9 16 20 24]") && ret;

      gmath::Tensord T3(gmath::Vectord({2}));

      ret=check("Operator /", T1/T3, "{2 3}[0.5 1 1.5 2 2.5 3]") && ret;

      gmath::Tensord T4(gmath::Vectord({1, 2, 3, 4}));
      T4.setReshaped({2, 2, 1});

      T1+=T4;
      ret=check("Operator +=", T1, "{2 2 3}[2 3 4 6 7 8 4 5 6 8 9 10]") && ret;

      T1-=T1;
      ret=check("Operator -=", T1, "{2 2 3}[0 0 0 0 0 0 0 0 0 0 0 0]") && ret;
    }



    std::cout << "--- Testing apply ---" << std::endl;

    {
      gmath::Tensord T1(gmath::Vectord({2*2, 3*3, 4*4, 5*5}));
      T1.apply(sqrt);
      ret=check("Apply", T1, "{4}[2 3 4 5]") && ret;
    }



    std::cout << "--- Testing min, max, sum and average operations ---" << std::endl;

    {
      gmath::Tensord T(gmath::Vectord({2, 6, 5, 4, 2, 3, 4, 3, 7, 3, 2, 1}));
      T.setReshaped({2, 2, 3});

      if (T.min() != 1)
      {
        std::cout << "Unexpected result: min: " << T.min() << " != 1" << std::endl;
        ret=false;
      }

      if (T.max() != 7)
      {
        std::cout << "Unexpected result: max: " << T.max() << " != 7" << std::endl;
        ret=false;
      }

      if (T.sum() != 42)
      {
        std::cout << "Unexpected result: sum: " << T.sum() << " != 42" << std::endl;
        ret=false;
      }

      if (T.average() != 3.5)
      {
        std::cout << "Unexpected result: average: " << T.average() << " != 3.5" << std::endl;
        ret=false;
      }

      ret=check("min(-1)", T.min(-1), "{2 2 1}[2 2 3 1]") && ret;
      ret=check("max(-2)", T.max(-2), "{2 1 3}[4 6 5 4 3 7]") && ret;
      ret=check("sum(-3)", T.sum(-3), "{1 2 3}[6 9 12 7 4 4]") && ret;
      ret=check("average(-2)", T.average(-2), "{2 1 3}[3 4 4 3.5 2.5 4]") && ret;
    }



    std::cout << "--- Testing matmul ---" << std::endl;

    {
      gmath::Tensord A(gmath::Vectord({1, 2}));
      gmath::Tensord B(gmath::Vectord({3, 4}));

      ret=check("Operator matmul 1", matmul(A, B), "{1 1}[11]") && ret;
      ret=check("Operator matmul 2", matmul(A.reshape({2, 1}), B.reshape({1, 2})), "{2 2}[3 4 6 8]") && ret;

      gmath::Tensord C(gmath::Vectord({1, 2, 3, 4, 5, 6}));
      C.setReshaped({2, 3});

      ret=check("Operator matmul 3", matmul(C.transpose({1, 0}), A), "{3 1}[9 12 15]") && ret;

      gmath::Tensord D(gmath::Vectord({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}));
      D.setReshaped({4, 3});

      ret=check("Operator matmul 4", matmul(C, D.transpose({1, 0})), "{2 4}[14 32 50 68 32 77 122 167]") && ret;

      D.setReshaped({2, 3, 2});

      ret=check("Operator matmul 5", matmul(C, D), "{2 2 2}[22 28 49 64 58 64 139 154]") && ret;
    }



    std::cout << "--- Testing input / output ---" << std::endl;

    {
      std::string sin="{4 2 3}[1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24]";
      std::istringstream in(sin);

      gmath::Tensord T;
      in >> T;

      if (!in)
      {
        std::cout << "Unexpected result: Reading from string failed" << std::endl;
        ret=false;
      }

      ret=check("Read", T, "{4 2 3}[1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24]") && ret;
    }
  }
  catch (const std::exception &ex)
  {
    std::cout << ex.what() << std::endl;
    ret=false;
  }

  if (ret)
  {
    std::cout << "Success" << std::endl;
  }
  else
  {
    std::cout << std::endl;
    std::cout << "Failed!" << std::endl;
  }

  return 0;
}
