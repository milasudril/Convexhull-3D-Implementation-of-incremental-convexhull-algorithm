/*The MIT License

Copyright (c) 2020 Dung-Han Lee
dunghanlee@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE. */

/* This work is an implementation of incremental convex hull algorithm from
the book Computational Geometry in C by O'Rourke */

//@ {"target":{"name":"convexhull.o"}}

#include "./convexhull.hpp"

#include <stdexcept>

void ConvexHull::AddOneFace(vertex_index a, vertex_index b, vertex_index c, const Point3D& inner_pt)
{
  // Make sure face is CCW with face normal pointing outward
  this->faces.emplace_back(Face{a, b, c});
  auto& new_face = this->faces.back();
  if(VolumeSign(std::data(std::as_const(*this).pointcloud),
    std::as_const(new_face), inner_pt) < 0)
  { new_face.flip(); }

  // Create edges and link them to face pointer
  auto create_edge = [&](vertex_index p1, vertex_index p2)
  {
    auto const i = edges.insert(std::pair{Edge{p1, p2}, EdgeData{}});
    i.first->second.LinkAdjFace(&new_face);
  };
  create_edge(a, b);
  create_edge(a, c);
  create_edge(b, c);
}

void ConvexHull::AddOneFace(std::pair<Edge const, EdgeData>& current_edge, vertex_index c, const Point3D& inner_pt)
{
  auto const a = current_edge.first.endpoints[0];
  auto const b = current_edge.first.endpoints[1];

  // Make sure face is CCW with face normal pointing outward
  this->faces.emplace_back(Face{a, b, c});
  auto& new_face = this->faces.back();
  if(VolumeSign(std::data(std::as_const(*this).pointcloud),
    std::as_const(new_face), inner_pt) < 0)
  { new_face.flip(); }

  // Create edges and link them to face pointer
  auto create_edge = [&](vertex_index p1, vertex_index p2)
  {
    auto const i = edges.insert(std::pair{Edge{p1, p2}, EdgeData{}});
    i.first->second.LinkAdjFace(&new_face);
  };

  current_edge.second.LinkAdjFace(&new_face);
  create_edge(a, c);
  create_edge(b, c);
}

void ConvexHull::BuildFirstHull(std::span<Point3D> pointcloud)
{
  auto const n = pointcloud.size();
  if(n <= 3)
  { throw std::runtime_error{"To few points in input data"}; }

  uint32_t i = 2;
  while(Colinear(pointcloud[i], pointcloud[i - 1], pointcloud[i - 2]))
  {
    if(i++ == n - 1)
    { throw std::runtime_error{"All points are colinear"}; }
  }

  Face const face{vertex_index{i}, vertex_index{i - 1}, vertex_index{i - 2}};

  auto j = i;
  while(!VolumeSign(std::data(std::as_const(*this).pointcloud), face, pointcloud[j]))
  {
    if(j++ == n-1)
    { throw std::runtime_error{"All pointcloud are coplanar"}; }
  }

  auto& p1 = pointcloud[i];    auto& p2 = pointcloud[i - 1];
  auto& p3 = pointcloud[i - 2];  auto& p4 = pointcloud[j];
  p1.processed = p2.processed = p3.processed = p4.processed = true;
  this->AddOneFace(vertex_index{i}, vertex_index{i - 1}, vertex_index{i - 2}, p4);
  this->AddOneFace(vertex_index{i}, vertex_index{i - 1}, vertex_index{j}, p3);
  this->AddOneFace(vertex_index{i}, vertex_index{i - 2}, vertex_index{j}, p2);
  this->AddOneFace(vertex_index{i - 1}, vertex_index{i - 2}, vertex_index{j}, p1);
}

size_t mark_visible_faces(std::list<Face>& faces, std::span<Point3D const> points, Point3D const& ref)
{
  size_t ret = 0;

  for(auto& face : faces)
  {
    if(VolumeSign(std::data(points), face, ref) < 0)
    {
      ++ret;
      face.visible = true;
    }
  }

  return ret;
}

void ConvexHull::IncreHull(const Point3D& pt)
{
  if(mark_visible_faces(faces, pointcloud, pt) == 0)
  { return; }


  // Find the edges to make new tangent surface or to be removed
  for(auto it = this->edges.begin(); it != this->edges.end(); it++)
  {
    auto& edge = *it;
    auto& face1 = edge.second.adjface1;
    auto& face2 = edge.second.adjface2;

    // Newly added edge
    if(face1 == nullptr || face2 == nullptr)
    { continue; }

    // This edge is to be removed because two adjacent faces will be removed
    else if(face1->visible && face2->visible)
    { edge.second.to_be_removed = true; }

    // Edge on the boundary of visibility, which will be used to extend a tangent
    // cone surface.
    else if(face1->visible || face2->visible)
    {
      if(face1->visible)
      { std::swap(face1, face2); }
      auto const inner_pt = FindInnerPoint(*face2, edge.first);
      edge.second.Erase(face2);
      this->AddOneFace(edge,
        vertex_index{&pt, std::data(std::as_const(*this).pointcloud)},
        *(std::data(std::as_const(*this).pointcloud) + inner_pt));
    }
  }
}

void ConvexHull::ConstructHull(std::span<Point3D> pointcloud)
{
  this->BuildFirstHull(pointcloud);

  for(const auto& pt : pointcloud)
  {
    if(pt.processed) continue;
    this->IncreHull(pt);
    this->CleanUp();
  }
}

void ConvexHull::CleanUp()
{
  auto it_edge = this->edges.begin();
  while(it_edge != this->edges.end())
  {
    if(it_edge->second.to_be_removed)
    { it_edge = this->edges.erase(it_edge); }
    else
    { ++it_edge; }
  }

  auto it_face = this->faces.begin();
  while(it_face != this->faces.end())
  {
    if(it_face->visible)
    {
      it_face = this->faces.erase(it_face);
    }
    else
    {
      ++it_face;
    }
  }
}