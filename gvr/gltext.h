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

#ifndef GVR_GLTEXT_H
#define GVR_GLTEXT_H

#include "globject.h"

#include <gmath/smatrix.h>
#include <gmath/svector.h>

#include <string>
#include <vector>

#include <GL/glew.h>

namespace gvr
{

class GLTextItem
{
  private:

    std::string  text;
    GLfloat trans[16];

  public:

    GLTextItem(const std::string &t, const gmath::Matrix33d &R, const gmath::Vector3d &T, double s,
               bool center=false);

    const std::string &getText() const { return text; }

    const GLfloat *getGLTransformation() const { return trans; }
};

class GLText : public GLObject
{
  private:

    std::vector<GLTextItem> list;

    GLText(const GLText &);
    GLText &operator=(const GLText &);

  public:

    GLText(int id) : GLObject(id) { }

    void addText(const std::string &text, const gmath::Matrix33d &R, const gmath::Vector3d &T,
                 double scale, bool center=false);

    virtual void draw(const GLCamera &cam);
};

}

#endif
