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

#include "coloredlines.h"

#ifdef INCLUDE_GL
#include "glcoloredlines.h"
#endif

#include <gutil/exception.h>

using std::vector;
using std::min;

namespace gvr
{

ColoredLines::ColoredLines()
{
    n=0;
    line=0;
}

ColoredLines::~ColoredLines()
{
    delete [] line;
}

void ColoredLines::resizeVertexList(int vn, bool with_scanprop, bool with_scanpos)
{
    ColoredPointCloud::resizeVertexList(vn, with_scanprop, with_scanpos);
}

void ColoredLines::resizeLineList(int ln)
{
    unsigned int *p=new unsigned int [(ln<<1)];
    
    for (int i=2*min(n, ln)-1; i>=0; i--)
      p[i]=line[i];
    
    delete [] line;
    
    line=p;
    n=ln;
}

void ColoredLines::addGLObjects(vector<GLObject*> &list)
{
#ifdef INCLUDE_GL
    list.push_back(new GLColoredLines(*this));
#else
    assert(false);
#endif
}

void ColoredLines::loadPLY(PLYReader &ply)
{
    assert(false);
}

void ColoredLines::savePLY(const char *name, bool all, ply_encoding enc) const
{
    assert(false);
}

}
