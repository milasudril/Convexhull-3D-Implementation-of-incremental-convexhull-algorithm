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

#include "./utility.hpp"

#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <span>

// Defined in CCW
struct Face
{
  Face(const Point3D& p1, const Point3D& p2, const Point3D& p3): visible(false)
      { vertices[0] = p1; vertices[1] = p2; vertices[2] = p3;};

  void Reverse(){std::swap(vertices[0], vertices[2]); };

  bool visible;
  Point3D vertices[3];
};

struct Edge
{
  Edge(const Point3D& p1, const Point3D& p2):
      adjface1(nullptr), adjface2(nullptr), remove(false)
      { endpoints[0] = p1; endpoints[1] = p2; };

  void LinkAdjFace(Face* face)
  {
    if( adjface1 != NULL && adjface2 != NULL )
    {
      std::cout<<"warning: property violated!\n";
      abort();
    }

    if(adjface1 == nullptr)
    { adjface1 = face; }
    else
    { adjface2 = face; }

  };

  void Erase(Face* face)
  {
    if(adjface1 != face && adjface2 != face) return;
    (adjface1 == face ? adjface1 : adjface2) = nullptr;
  };

  Face* adjface1;
  Face* adjface2;
  bool remove;
  Point3D endpoints[2];
};

class ConvexHull
{
  public:
    template<typename T> ConvexHull(const std::vector<T>& points);
    // All major works are conducted upon calling of constructor

    ~ConvexHull() = default;

    template<typename T> bool Contains(T p) const;
    // In out test for a query point (surface point is considered outside)

    const std::list<Face>& GetFaces() const {return this->faces;};

    const std::vector<Point3D>& GetVertices() const \
        {return this->exterior_points;};
    // Return exterior vertices than defines the convell hull

    size_t Size() const {return this->exterior_points.size();};

  private:

    bool Colinear(const Point3D& a, const Point3D& b, const Point3D& c) const;

    bool CoPlanar(Face& f, Point3D& p);

    int VolumeSign(const Face& f, const Point3D& p) const;
    // A point is considered outside of a CCW face if the volume of tetrahedron
    // formed by the face and point is negative. Note that origin is set at p.

    size_t Key2Edge(const Point3D& a, const Point3D& b) const;
    // Hash key for edge. hash(a, b) = hash(b, a)

    void AddOneFace(const Point3D& a, const Point3D& b,
        const Point3D& c, const Point3D& inner_pt);
    // Inner point is used to make the orientation of face consistent in counter-
    // clockwise direction

    bool BuildFirstHull(std::span<Point3D> pointcloud);
    // Build a tetrahedron as first convex hull

    void IncreHull(const Point3D& p);

    void ConstructHull(std::span<Point3D> pointcloud);

    void CleanUp();

    void ExtractExteriorPoints();

    Point3D FindInnerPoint(const Face* f, const Edge& e);
    // for face(a,b,c) and edge(a,c), return b

    std::vector<Point3D> pointcloud = {};
    std::vector<Point3D> exterior_points = {};
    std::list<Face> faces = {};
    std::list<Edge> edges = {};

    struct EdgeEndpoints
    {
      Point3D p1;
      Point3D p2;

      constexpr bool operator==(const EdgeEndpoints&) const = default;
      constexpr bool operator!=(const EdgeEndpoints&) const = default;
    };

    struct EdgeEndpointHash
    {
      size_t operator()(EdgeEndpoints const& a) const
      {
        return PointHash{}(a.p1) ^ PointHash{}(a.p2);
      }
    };

    std::unordered_map<EdgeEndpoints, Edge*, EdgeEndpointHash> map_edges;
};

template<typename T> ConvexHull::ConvexHull(const std::vector<T>& points)
{
  auto const n = points.size();
  this->pointcloud.resize(n);
  for(size_t i = 0; i != n; i++)
  {
    this->pointcloud[i].x = points[i].x;
    this->pointcloud[i].y = points[i].y;
    this->pointcloud[i].z = points[i].z;
  }
  this->ConstructHull(this->pointcloud);
}

template<typename T> bool ConvexHull::Contains(T p) const
{
  Point3D pt(p.x, p.y, p.z);
  for(auto& face : this->faces)
  {
    if(VolumeSign(face, pt) <= 0) return false;
  }
  return true;
}
#endif