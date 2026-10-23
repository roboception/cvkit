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

#include "tensor.h"

#include <sstream>

namespace gmath
{

void broadcast(std::vector<size_t> &shape1, std::vector<size_t> &step1,
  std::vector<size_t> &shape2, std::vector<size_t> &step2)
{
  // equalize rank by adding higher dimensions of size 1

  while (shape1.size() < shape2.size())
  {
    shape1.insert(shape1.begin(), 1);
    step1.insert(step1.begin(), 0);
  }

  while (shape2.size() < shape1.size())
  {
    shape2.insert(shape2.begin(), 1);
    step2.insert(step2.begin(), 0);
  }

  for (size_t i=0; i<shape1.size(); i++)
  {
    // for all dimensions with size 1, set step to 0 to disable indexing them,
    // just in case the corresponding size of the other shape is larger than 1

    if (shape1[i] == 1 && shape2[i] > 1) step1[i]=0;
    if (shape2[i] == 1 && shape1[i] > 1) step2[i]=0;

    // adapt shapes to each other

    if (shape1[i] != shape2[i])
    {
      if (shape1[i] == 1 || shape2[i] == 1)
      {
        shape1[i]=shape2[i]=std::max(shape1[i], shape2[i]);
      }
      else
      {
        std::ostringstream out;

        out << "Cannot broadcast non-conforming shapes: {";

        for (size_t i=0; i<shape1.size(); i++)
        {
          out << shape1[i];
          if (i+1 < shape1.size()) out << ' ';
        }

        out << "} != {";

        for (size_t i=0; i<shape2.size(); i++)
        {
          out << shape2[i];
          if (i+1 < shape2.size()) out << ' ';
        }

        out << "}";

        throw std::invalid_argument(out.str());
      }
    }
  }
}

}
