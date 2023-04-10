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

//@ {"target":{"name":"builder.o"}}

#include "./builder.hpp"

#include <stdexcept>

void convhull::builder::insert_face(point_3d const* vert_array, vertex_index a, vertex_index b, vertex_index c, point_3d ref)
{
  m_faces.emplace_back(make_oriented_face(vert_array, a, b, c, ref));
  auto& new_face = m_faces.back();

  create_and_link_edge(m_edges, a, b, new_face);
  create_and_link_edge(m_edges, a, c, new_face);
  create_and_link_edge(m_edges, b, c, new_face);
}

void convhull::builder::insert_face(point_3d const* vert_array,
  std::pair<edge const, edge_data>& current_edge,
  vertex_index c,
  point_3d ref)
{
  auto const a = current_edge.first.endpoints[0];
  auto const b = current_edge.first.endpoints[1];

  m_faces.emplace_back(make_oriented_face(vert_array, a, b, c, ref));
  auto& new_face = m_faces.back();

  current_edge.second.link_face(&new_face);
  create_and_link_edge(m_edges, a, c, new_face);
  create_and_link_edge(m_edges, b, c, new_face);
}

void convhull::builder::create_seed(std::span<point_3d const> points)
{
  auto const n = points.size();
  if(n <= 3)
  { throw std::runtime_error{"To few points in input data"}; }

  uint32_t i = 2;
  while(colinear(points[i], points[i - 1], points[i - 2]))
  {
    if(i++ == n - 1)
    { throw std::runtime_error{"All points are colinear"}; }
  }

  face const face{vertex_index{i}, vertex_index{i - 1}, vertex_index{i - 2}};

  auto const vert_array = std::data(points);

  auto j = i;
  while(!VolumeSign(vert_array, face, points[j]))
  {
    if(j++ == n-1)
    { throw std::runtime_error{"All points are coplanar"}; }
  }

  auto p1 = points[i];
  auto p2 = points[i - 1];
  auto p3 = points[i - 2];
  auto p4 = points[j];

  m_visited[i] = 1;
  m_visited[i - 1] = 1;
  m_visited[i - 2] = 1;
  m_visited[j] = 1;

  insert_face(vert_array, vertex_index{i}, vertex_index{i - 1}, vertex_index{i - 2}, p4);
  insert_face(vert_array, vertex_index{i}, vertex_index{i - 1}, vertex_index{j}, p3);
  insert_face(vert_array, vertex_index{i}, vertex_index{i - 2}, vertex_index{j}, p2);
  insert_face(vert_array, vertex_index{i - 1}, vertex_index{i - 2}, vertex_index{j}, p1);
}

size_t convhull::mark_visible_faces(face_list& faces, point_3d const* points, point_3d cam_loc)
{
  size_t ret = 0;

  for(auto& face : faces)
  {
    if(VolumeSign(points, face, cam_loc) < 0)
    {
      ++ret;
      face.visible = true;
    }
  }

  return ret;
}

void convhull::builder::try_insert(point_3d const* vert_array, std::reference_wrapper<point_3d const> pt)
{
  if(mark_visible_faces(m_faces, vert_array, pt) == 0)
  { return; }

  // Find the edges to make new tangent surface or to be removed
  for(auto& edge: m_edges)
  {
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
      auto const inner_pt = find_inner_point(*face2, edge.first);
      edge.second.erase(face2);
      insert_face(vert_array, edge, vertex_index{&pt.get(), vert_array}, *(vert_array + inner_pt));
    }
  }
}

void convhull::cleanup(edge_map& edges)
{
  auto it_edge = edges.begin();
  while(it_edge != edges.end())
  {
    if(it_edge->second.to_be_removed)
    { it_edge = edges.erase(it_edge); }
    else
    { ++it_edge; }
  }
}

void convhull::remove_hidden(face_list& faces)
{
  auto it_face = faces.begin();
  while(it_face != faces.end())
  {
    if(it_face->visible)
    { it_face = faces.erase(it_face); }
    else
    { ++it_face; }
  }
}

void convhull::builder::create(std::span<point_3d const> points)
{
  create_seed(points);

  for(auto const& pt : points)
  {
    if(m_visited[vertex_index{&pt, std::data(points)}.value()])
    { continue; }

    try_insert(std::data(points), pt);
    cleanup(m_edges);
    remove_hidden(m_faces);
  }
}