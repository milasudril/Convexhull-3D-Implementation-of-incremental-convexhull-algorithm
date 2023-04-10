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

void ConvexHull::insert_face(Point3D const* vert_array, vertex_index a, vertex_index b, vertex_index c, const Point3D& ref)
{
  m_faces.emplace_back(make_oriented_face(vert_array, a, b, c, ref));
  auto& new_face = m_faces.back();

  create_and_link_edge(m_edges, a, b, new_face);
  create_and_link_edge(m_edges, a, c, new_face);
  create_and_link_edge(m_edges, b, c, new_face);
}

void ConvexHull::insert_face(Point3D const* vert_array, std::pair<edge const, edgeData>& current_edge,
  vertex_index c,
  const Point3D& ref)
{
  auto const a = current_edge.first.endpoints[0];
  auto const b = current_edge.first.endpoints[1];

  m_faces.emplace_back(make_oriented_face(vert_array, a, b, c, ref));
  auto& new_face = m_faces.back();

  current_edge.second.link_face(&new_face);
  create_and_link_edge(m_edges, a, c, new_face);
  create_and_link_edge(m_edges, b, c, new_face);
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

  face const face{vertex_index{i}, vertex_index{i - 1}, vertex_index{i - 2}};

  auto const vert_array = std::data(pointcloud);

  auto j = i;
  while(!VolumeSign(vert_array, face, pointcloud[j]))
  {
    if(j++ == n-1)
    { throw std::runtime_error{"All pointcloud are coplanar"}; }
  }

  auto& p1 = pointcloud[i];    auto& p2 = pointcloud[i - 1];
  auto& p3 = pointcloud[i - 2];  auto& p4 = pointcloud[j];
  p1.processed = p2.processed = p3.processed = p4.processed = true;
  this->insert_face(vert_array, vertex_index{i}, vertex_index{i - 1}, vertex_index{i - 2}, p4);
  this->insert_face(vert_array, vertex_index{i}, vertex_index{i - 1}, vertex_index{j}, p3);
  this->insert_face(vert_array, vertex_index{i}, vertex_index{i - 2}, vertex_index{j}, p2);
  this->insert_face(vert_array, vertex_index{i - 1}, vertex_index{i - 2}, vertex_index{j}, p1);
}

size_t mark_visible_faces(std::list<face>& faces, std::span<Point3D const> points, Point3D const& ref)
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
  if(mark_visible_faces(m_faces, m_vertices, pt) == 0)
  { return; }

  auto const vert_array = std::data(m_vertices);

  // Find the edges to make new tangent surface or to be removed
  for(auto it = m_edges.begin(); it != m_edges.end(); ++it)
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

    // edge on the boundary of visibility, which will be used to extend a tangent
    // cone surface.
    else if(face1->visible || face2->visible)
    {
      if(face1->visible)
      { std::swap(face1, face2); }
      auto const inner_pt = FindInnerPoint(*face2, edge.first);
      edge.second.erase(face2);
      this->insert_face(
        vert_array,
        edge,
        vertex_index{&pt, vert_array},
        *(vert_array + inner_pt));
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
  auto it_edge = m_edges.begin();
  while(it_edge != m_edges.end())
  {
    if(it_edge->second.to_be_removed)
    { it_edge = m_edges.erase(it_edge); }
    else
    { ++it_edge; }
  }

  auto it_face = m_faces.begin();
  while(it_face != m_faces.end())
  {
    if(it_face->visible)
    {
      it_face = m_faces.erase(it_face);
    }
    else
    {
      ++it_face;
    }
  }
}