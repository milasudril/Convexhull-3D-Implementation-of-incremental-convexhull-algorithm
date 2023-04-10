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

//@ {"dependencies_extra":[{"ref":"./convexhull.o", "rel":"implementation"}]}

#ifndef CONVEXHULL_HPP
#define CONVEXHULL_HPP

#include "./point.hpp"

#include <vector>
#include <string>
#include <map>
#include <list>
#include <span>
#include <bit>
#include <cassert>
#include <functional>

class vertex_index
{
public:
  constexpr explicit vertex_index(uint32_t val):m_value{val}{}

  constexpr explicit vertex_index(point_3d const* ptr, point_3d const* base):
    m_value{static_cast<uint32_t>(ptr - base)}
  {}

  constexpr bool operator==(vertex_index const&) const = default;
  constexpr bool operator!=(vertex_index const&) const = default;

  constexpr auto value() const { return m_value; }

private:
  uint32_t m_value;
};

constexpr auto operator+(point_3d* ptr, vertex_index vi)
{
  return ptr + vi.value();
}

constexpr auto operator+(point_3d const* ptr, vertex_index vi)
{
  return ptr + vi.value();
}


struct face
{
  std::array<vertex_index, 3> vertices;
  bool visible{false};

  void flip()
  { std::swap(vertices[0], vertices[2]); }
};

// A point is considered outside of a CCW face if the volume of tetrahedron
// formed by the face and point is negative. Note that origin is set at p.
inline auto VolumeSign(point_3d const* vert_array, face const& f, point_3d p)
{
  std::array<point_3d, 3> const vertices{
    *(vert_array + f.vertices[0]),
    *(vert_array + f.vertices[1]),
    *(vert_array + f.vertices[2])
  };

  auto const ax = vertices[0].x - p.x;
  auto const ay = vertices[0].y - p.y;
  auto const az = vertices[0].z - p.z;
  auto const bx = vertices[1].x - p.x;
  auto const by = vertices[1].y - p.y;
  auto const bz = vertices[1].z - p.z;
  auto const cx = vertices[2].x - p.x;
  auto const cy = vertices[2].y - p.y;
  auto const cz = vertices[2].z - p.z;
  auto const vol = ax * (by * cz - bz * cy) +\
        ay * (bz * cx - bx * cz) +\
        az * (bx * cy - by * cx);
  if(vol == 0) return 0;
  return vol < 0 ? -1 : 1;
}

inline auto make_oriented_face(point_3d const* vert_array,
  vertex_index a,
  vertex_index b,
  vertex_index c,
  point_3d ref)
{
  face ret{a, b, c};
  if(VolumeSign(vert_array, ret, ref) < 0)
  { ret.flip(); }

  return ret;
}

struct edge
{
  constexpr explicit edge(vertex_index p1, vertex_index p2):
    endpoints{p1, p2}
  {}

  std::array<vertex_index, 2> endpoints;

  constexpr bool operator==(edge const&) const = default;
  constexpr bool operator!=(edge const&) const = default;
};

struct edge_data
{
  explicit edge_data():
    adjface1{nullptr},
    adjface2{nullptr},
    to_be_removed{false}
  {}

  void link_face(face* face)
  {
    assert(adjface1 == nullptr || adjface2 == nullptr);

    if(adjface1 == nullptr)
    { adjface1 = face; }
    else
    { adjface2 = face; }
  }

  void erase(face const* face)
  {
    assert(adjface1 == face || adjface2 == face);

    if(adjface1 == face)
    { adjface1 = nullptr; }
    else
    { adjface2 = nullptr; }
  }

  face* adjface1;
  face* adjface2;
  bool to_be_removed;
};

struct edge_cmp
{
  size_t operator()(edge const& a, edge const& b) const
  {
    auto const a_bits = std::bit_cast<size_t>(a);
    auto const b_bits = std::bit_cast<size_t>(b);
    return a_bits < b_bits;
  }
};

using edge_map = std::map<edge, edge_data, edge_cmp>;

inline void create_and_link_edge(edge_map& edges, vertex_index p1, vertex_index p2, face& face)
{
  auto const i = edges.insert(std::pair{edge{p1, p2}, edge_data{}});
  i.first->second.link_face(&face);
}

// for face(a,b,c) and edge(a,c), return b
inline auto find_inner_point(const face& f, const edge& e)
{
  for(size_t i = 0; i != std::size(f.vertices); i++)
  {
    if(f.vertices[i] == e.endpoints[0]) continue;
    if(f.vertices[i] == e.endpoints[1]) continue;
    return f.vertices[i];
  }

  abort();
}

class ConvexHull
{
  public:
    explicit ConvexHull(std::span<point_3d const> points):
      m_visited(std::size(points), 0)
    { create(points); }

    std::list<face> const& faces() const {return m_faces;}

  private:
    void insert_face(point_3d const* vert_array, vertex_index a, vertex_index b, vertex_index c, point_3d inner_pt);
    void insert_face(point_3d const* vert_array, std::pair<edge const, edge_data>& current_edge, vertex_index c, point_3d inner_pt);

    void create_seed(std::span<point_3d const> pointcloud);
    void try_insert(point_3d const* vert_array, std::reference_wrapper<point_3d const> p);
    void create(std::span<point_3d const> pointcloud);

    std::vector<int8_t> m_visited;
    std::list<face> m_faces;
    edge_map m_edges;
};

#endif