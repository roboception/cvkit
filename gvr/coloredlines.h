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

#ifndef GVR_COLOREDLINES_H
#define GVR_COLOREDLINES_H

#include "coloredpointcloud.h"

namespace gvr
{

class ColoredLines : public ColoredPointCloud
{
  private:
  
    int n;
    unsigned int *line;
    
    ColoredLines(const ColoredLines &);
    ColoredLines& operator=(const ColoredLines &);
    
  public:
    
    ColoredLines();
    virtual ~ColoredLines();
    
    virtual void resizeVertexList(int vn, bool with_scanprop, bool with_scanpos);
    virtual void resizeLineList(int ln);
    
    int getLineCount() const { return n; }
    
    unsigned int *getLineArray() { return line; }
    unsigned int getLineIndex(int i, int c) const { return line[(i<<1)+c]; }
    
    void setLineIndex(int i, unsigned int a, unsigned int b)
    {
      line[(i<<1)]=a;
      line[(i<<1)+1]=b;
    }
    
    virtual void addGLObjects(std::vector<GLObject*> &list);
    
    virtual void loadPLY(PLYReader &ply);
    virtual void savePLY(const char *name, bool all=true, ply_encoding enc=ply_binary) const;
};

}

#endif
