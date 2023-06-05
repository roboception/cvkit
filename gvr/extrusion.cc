/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2023 Roboception GmbH
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

#include "extrusion.h"

#include <stdexcept>

#ifdef INCLUDE_GLU

#include <GL/glu.h>

#include <memory>

namespace gvr
{

namespace
{

struct VertexData
{
  GLdouble xyz[3];
  int      index;  // vertex index
};

struct TriangleData
{
  int index[3]; // indices of triangle vertices

  TriangleData()
  {
    index[0]=-1;
    index[1]=-1;
    index[2]=-1;
  }

  TriangleData(int a, int b, int c)
  {
    index[0]=a;
    index[1]=b;
    index[2]=c;
  }
};

struct MeshData
{
  int nvertex; // total number of vertices

  // contours / vertices / index, x, y, z

  std::vector<std::vector<VertexData> > vertex;

  // possible additional vertices due to tessalation (this may be extended
  // during tesselation in combineCb, while pointers to vertex data are held,
  // therefore this is done with shared pointers in contrast to the static
  // vertex arrays above)

  std::vector<std::shared_ptr<VertexData> > extra_vertex;

  // type of triangle order in tessalation and indices that are used depending
  // on that type

  GLenum type;
  int tindex0;
  int tindex1;
  bool inverse;

  // triangle with indices to vertices

  std::vector<TriangleData> triangle;

  MeshData()
  {
    nvertex=0;
    type=GL_TRIANGLES;
    tindex0=-1;
    tindex1=-1;
    inverse=false;
  }
};

void beginCb(GLenum type, void *user_data)
{
  MeshData *md=reinterpret_cast<MeshData *>(user_data);

  // tesselation starts
  md->type=type;
  md->tindex0=-1;
  md->tindex1=-1;
  md->inverse=false;
}

void combineCb(GLdouble coords[3], void *[4], GLfloat [4], void **out_data, void *user_data)
{
  MeshData *md=reinterpret_cast<MeshData *>(user_data);

  // tesselation needs to create a new point

  std::shared_ptr<VertexData> vd=std::make_shared<VertexData>();

  vd->index=md->nvertex++;
  vd->xyz[0]=coords[0];
  vd->xyz[1]=coords[1];
  vd->xyz[2]=coords[2];

  md->extra_vertex.push_back(vd);

  *out_data=vd.get();
}

void vertexCb(void *vertex_data, void *user_data)
{
  MeshData *md=reinterpret_cast<MeshData *>(user_data);
  VertexData *vd=reinterpret_cast<VertexData *>(vertex_data);

  // create triangles from vertex according to type

  switch (md->type)
  {
    case GL_TRIANGLES: // Triangles: {0 1 2} {3 4 5} ...
      if (md->tindex0 < 0)
      {
        md->tindex0=vd->index;
      }
      else if (md->tindex1 < 0)
      {
        md->tindex1=vd->index;
      }
      else
      {
        md->triangle.push_back(TriangleData(md->tindex0, md->tindex1, vd->index));
        md->tindex0=-1;
        md->tindex1=-1;
      }
      break;

    case GL_TRIANGLE_STRIP: // Triangles: {0 1 2} {2 1 3} {2 3 4} {4 3 5} ...
      if (md->tindex0 < 0)
      {
        md->tindex0=vd->index;
      }
      else if (md->tindex1 < 0)
      {
        md->tindex1=vd->index;
      }
      else
      {
        if (md->inverse)
        {
          md->triangle.push_back(TriangleData(md->tindex1, md->tindex0, vd->index));
        }
        else
        {
          md->triangle.push_back(TriangleData(md->tindex0, md->tindex1, vd->index));
        }

        md->tindex0=md->tindex1;
        md->tindex1=vd->index;
        md->inverse=!md->inverse;
      }
      break;

    case GL_TRIANGLE_FAN: // Triangle: {0 1 2} {0 2 3} {0 3 4} {0 4 5} ...
      if (md->tindex0 < 0)
      {
        md->tindex0=vd->index;
      }
      else if (md->tindex1 < 0)
      {
        md->tindex1=vd->index;
      }
      else
      {
        md->triangle.push_back(TriangleData(md->tindex0, md->tindex1, vd->index));
        md->tindex1=vd->index;
      }
      break;

    default:
      std::cerr << "Internal error: Unknown type" << std::endl;
      break;
  }
}

void endCb(void *)
{
  // nothing to be done here
}

}

void createMeshFromContours(Mesh *mesh, const std::vector<gimage::Polygon> &contour,
  double scale, double height)
{
  // there must be at least one contour

  if (contour.size() == 0)
  {
    return;
  }

  // create list of vertices for all contours

  MeshData md;
  md.nvertex=0;
  md.vertex.resize(contour.size());

  // get vertices for all contours

  {
    // copy contours

    for (size_t i=0; i<contour.size(); i++)
    {
      md.vertex[i].resize(contour[i].count());
      for (int j=0; j<contour[i].count(); j++)
      {
        VertexData &vd=md.vertex[i][j];
        vd.index=md.nvertex++;
        vd.xyz[0]=scale*(contour[i].getX(j)+0.5);
        vd.xyz[1]=scale*(contour[i].getY(j)+0.5);
        vd.xyz[2]=0;
      }
    }
  }

  // triangulate outer contour with inner contours as holes for top face
  // using tesselation function from OpenGL utility library libGLU

  {
    GLUtesselator* tobj=gluNewTess();
    gluTessCallback(tobj, GLU_TESS_BEGIN_DATA, reinterpret_cast<GLvoid (*) ()>(&beginCb));
    gluTessCallback(tobj, GLU_TESS_COMBINE_DATA, reinterpret_cast<GLvoid (*) ()>(&combineCb));
    gluTessCallback(tobj, GLU_TESS_VERTEX_DATA, reinterpret_cast<GLvoid (*) ()>(&vertexCb));
    gluTessCallback(tobj, GLU_TESS_END_DATA, reinterpret_cast<GLvoid (*) ()>(&endCb));

    // outer contours counterclockwise, inner contours clockwise
    gluTessProperty(tobj, GLU_TESS_WINDING_RULE, GLU_TESS_WINDING_ODD);
    gluTessProperty(tobj, GLU_TESS_BOUNDARY_ONLY, GL_FALSE);
    gluTessProperty(tobj, GLU_TESS_TOLERANCE, 0);

    gluTessBeginPolygon(tobj, &md);

    for (size_t i=0; i<md.vertex.size(); i++)
    {
      gluTessBeginContour(tobj);

      for (size_t k=0; k<md.vertex[i].size(); k++)
      {
        gluTessVertex(tobj, md.vertex[i][k].xyz, &md.vertex[i][k]);
      }

      gluTessEndContour(tobj);
    }

    gluTessEndPolygon(tobj);
    gluDeleteTess(tobj);
  }

  // determine total number of vertices

  size_t vn=md.extra_vertex.size();
  for (size_t i=0; i<md.vertex.size(); i++)
  {
    vn+=md.vertex[i].size();
  }

  // create mesh object and set number of vertices and triangles

  mesh->resizeVertexList(static_cast<int>(4*vn), false, false);
  mesh->resizeTriangleList(static_cast<int>(2*md.triangle.size()+2*(vn-md.extra_vertex.size())));

  // copy vertices for top side

  int j=0;
  for (size_t i=0; i<md.vertex.size(); i++)
  {
    for (size_t k=0; k<md.vertex[i].size(); k++)
    {
      mesh->setVertex(j, static_cast<float>(md.vertex[i][k].xyz[0]),
        static_cast<float>(md.vertex[i][k].xyz[1]),
        static_cast<float>(md.vertex[i][k].xyz[2]));
      j++;
    }
  }

  for (size_t k=0; k<md.extra_vertex.size(); k++)
  {
    mesh->setVertex(j, static_cast<float>(md.extra_vertex[k]->xyz[0]),
      static_cast<float>(md.extra_vertex[k]->xyz[1]),
      static_cast<float>(md.extra_vertex[k]->xyz[2]));
    j++;
  }

  // duplicate vertices for bottom side with different z coordinate

  for (size_t i=0; i<vn; i++)
  {
    mesh->setVertex(j, mesh->getVertexComp(i, 0), mesh->getVertexComp(i, 1),
      static_cast<float>(height));
    j++;
  }

  // copy all vertices again for walls (this is necessary to make the normals
  // for top / bottom and walls are independent)

  for (size_t i=0; i<2*vn; i++)
  {
    mesh->setVertex(j, mesh->getVertexComp(i, 0), mesh->getVertexComp(i, 1),
      mesh->getVertexComp(i, 2));
    j++;
  }

  // copy triangles for top side

  j=0;
  for (size_t i=0; i<md.triangle.size(); i++)
  {
    mesh->setTriangleIndex(j++, md.triangle[i].index[0], md.triangle[i].index[1],
      md.triangle[i].index[2]);
  }

  // duplicate triangles for bottom side in inverse vertex order

  for (size_t i=0; i<md.triangle.size(); i++)
  {
    mesh->setTriangleIndex(j++, vn+md.triangle[i].index[1], vn+md.triangle[i].index[0],
      vn+md.triangle[i].index[2]);
  }

  // create triangles for walls

  int top=2*vn;
  int bottom=3*vn;

  for (size_t i=0; i<md.vertex.size(); i++)
  {
    size_t l=md.vertex[i].size()-1;
    for (size_t k=0; k<md.vertex[i].size(); k++)
    {
      mesh->setTriangleIndex(j++, top+md.vertex[i][l].index, bottom+md.vertex[i][l].index,
        top+md.vertex[i][k].index);
      mesh->setTriangleIndex(j++, top+md.vertex[i][k].index, bottom+md.vertex[i][l].index,
        bottom+md.vertex[i][k].index);
      l=k;
    }
  }

  // calculate normals and save ply

  mesh->recalculateNormals();
}

}

#else

namespace gvr
{

void createMeshFromContours(Mesh *mesh, const std::vector<gimage::Polygon> &pl,
  double scale, double height)
{
  throw std::invalid_argument("createMeshFromContours not implemented. Please recompile with GLU support");
}

}

#endif
