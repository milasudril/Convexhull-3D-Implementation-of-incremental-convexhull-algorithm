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

int ConvexHull::VolumeSign(const Face& f, const Point3D& p) const
{
  double vol;
  double ax, ay, az, bx, by, bz, cx, cy, cz;
  ax = f.vertices[0].x - p.x;
  ay = f.vertices[0].y - p.y;
  az = f.vertices[0].z - p.z;
  bx = f.vertices[1].x - p.x;
  by = f.vertices[1].y - p.y;
  bz = f.vertices[1].z - p.z;
  cx = f.vertices[2].x - p.x;
  cy = f.vertices[2].y - p.y;
  cz = f.vertices[2].z - p.z;
  vol = ax * (by * cz - bz * cy) +\
        ay * (bz * cx - bx * cz) +\
        az * (bx * cy - by * cx);
  if(vol == 0) return 0;
  return vol < 0 ? -1 : 1;
}

void ConvexHull::AddOneFace(const Point3D& a, const Point3D& b,
    const Point3D& c, const Point3D& inner_pt)
{
  // Make sure face is CCW with face normal pointing outward
  this->faces.emplace_back(a, b, c);
  auto& new_face = this->faces.back();
  if(this->VolumeSign(this->faces.back(), inner_pt) < 0) new_face.Reverse();

  // Create edges and link them to face pointer
  auto create_edge = [&](const Point3D& p1, const Point3D& p2)
  {
    auto const key = EdgeEndpoints{p1, p2};
    if(!this->map_edges.count(key))
    {
      this->edges.emplace_back(p1, p2);
      this->map_edges.insert({key, &this->edges.back()});
    }
    this->map_edges[key]->LinkAdjFace(&new_face);
  };
  create_edge(a, b);
  create_edge(a, c);
  create_edge(b, c);
}

bool ConvexHull::Colinear(const Point3D& a, const Point3D& b, const Point3D& c) const
{
  return ((c.z - a.z) * (b.y - a.y) -
          (b.z - a.z) * (c.y - a.y)) == 0 &&\
         ((b.z - a.z) * (c.x - a.x) -
          (b.x - a.x) * (c.z - a.z)) == 0 &&\
         ((b.x - a.x) * (c.y - a.y) -
          (b.y - a.y) * (c.x - a.x)) == 0;
}

bool ConvexHull::BuildFirstHull(std::span<Point3D> pointcloud)
{
  auto const n = pointcloud.size();
  if(n <= 3)
  {
    std::cout<<"Tetrahedron: points.size() < 4\n";
    return false;
  }

  size_t i = 2;
  while(this->Colinear(pointcloud[i], pointcloud[i - 1], pointcloud[i - 2]))
  {
    if(i++ == n - 1)
    {
      std::cout<<"Tetrahedron: All points are colinear!\n";
      return false;
    }
  }

  Face face(pointcloud[i], pointcloud[i-1], pointcloud[i-2]);

  auto j = i;
  while(!this->VolumeSign(face, pointcloud[j]))
  {
    if(j++ == n-1)
    {
      std::cout<<"Tetrahedron: All pointcloud are coplanar!\n";
      return false;
    }
  }

  auto& p1 = pointcloud[i];    auto& p2 = pointcloud[i-1];
  auto& p3 = pointcloud[i-2];  auto& p4 = pointcloud[j];
  p1.processed = p2.processed = p3.processed = p4.processed = true;
  this->AddOneFace(p1, p2, p3, p4);
  this->AddOneFace(p1, p2, p4, p3);
  this->AddOneFace(p1, p3, p4, p2);
  this->AddOneFace(p2, p3, p4, p1);
  return true;

}

Point3D ConvexHull::FindInnerPoint(const Face* f, const Edge& e)
{
  for(int i = 0; i < 3; i++)
  {
    if(f->vertices[i] == e.endpoints[0]) continue;
    if(f->vertices[i] == e.endpoints[1]) continue;
    return f->vertices[i];
  }

  abort();
}

void ConvexHull::IncreHull(const Point3D& pt)
{
  // Find the illuminated faces (which will be removed later)
  bool vis = false;
  for(auto& face : this->faces)
  {
    if(VolumeSign(face, pt) < 0)
    {
      vis = true;
      face.visible = true;
    }
  }
  if(!vis) return;

  // Find the edges to make new tagent surface or to be removed
  for(auto it = this->edges.begin(); it != this->edges.end(); it++)
  {
    auto& edge = *it;
    auto& face1 = edge.adjface1;
    auto& face2 = edge.adjface2;

    // Newly added edge
    if(face1 == NULL || face2 == NULL)
    {
      continue;
    }

    // This edge is to be removed because two adjacent faces will be removed
    else if(face1->visible && face2->visible)
    {
      edge.remove = true;
    }

    // Edge on the boundary of visibility, which will be used to extend a tagent
    // cone surface.
    else if(face1->visible|| face2->visible)
    {
      if(face1->visible) std::swap(face1, face2);
      auto inner_pt = this->FindInnerPoint(face2, edge);
      edge.Erase(face2);
      this->AddOneFace(edge.endpoints[0], edge.endpoints[1], pt, inner_pt);
    }
  }
}

void ConvexHull::ConstructHull(std::span<Point3D> pointcloud)
{
  if(!this->BuildFirstHull(pointcloud)) return;
  for(const auto& pt : pointcloud)
  {
    if(pt.processed) continue;
    this->IncreHull(pt);
    this->CleanUp();
  }
  this->ExtractExteriorPoints();
}

void ConvexHull::CleanUp()
{
  auto it_edge = this->edges.begin();
  while(it_edge != this->edges.end())
  {
    if(it_edge->remove)
    {
      auto pt1 = it_edge->endpoints[0];
      auto pt2 = it_edge->endpoints[1];
      auto key_to_evict = EdgeEndpoints{pt1, pt2};
      this->map_edges.erase(key_to_evict);
      this->edges.erase(it_edge++);
    }
    else it_edge++;
  };
  auto it_face = this->faces.begin();
  while(it_face != this->faces.end())
  {
    if(it_face->visible) this->faces.erase(it_face++);
    else it_face++;
  }
}


void ConvexHull::ExtractExteriorPoints()
{
  std::unordered_set<Point3D, PointHash> exterior_set;
  for(const auto& f : this->faces)
  {
    for(int i =0; i < 3; i++)
      exterior_set.insert(f.vertices[i]);
  }
  this->exterior_points = \
      std::vector<Point3D>(exterior_set.begin(), exterior_set.end());
}