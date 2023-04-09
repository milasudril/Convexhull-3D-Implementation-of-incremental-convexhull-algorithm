//@ {"target":{"name":"plain_demo.o"}}

#include "./convexhull.hpp"
#include <vector>
#include <array>
#include <cassert>
#include <algorithm>

struct face_indirect
{
  std::array<uint32_t, 3> vertices;
};

int main()
{
  std::vector<Point3D> tests = \
    {{3,0,0}, {0,3,0}, {0,0,3}, {3,3,3}};
  ConvexHull C(tests);

  std::unordered_map<Point3D, uint32_t, PointHash> vertex_index;
  std::vector<Point3D> sorted_verts;
  std::vector<face_indirect> faces;
  for(auto const& face : C.GetFaces())
  {
    face_indirect f{};
    for(size_t k = 0; k != std::size(f.vertices); ++k)
    {
      auto const vi = static_cast<uint32_t>(std::size(vertex_index));
      auto i = vertex_index.insert(std::pair{face.vertices[k], vi});
      if(i.second)
      { sorted_verts.push_back(face.vertices[k]); }
      f.vertices[k] = i.first->second;
    }
    faces.push_back(f);
  }

  assert(std::size(vertex_index) == std::size(sorted_verts));

  std::ranges::for_each(sorted_verts, [](auto const& item){
    printf("v %.8g %.8g %.8g\n", item.x, item.y, item.z);
  });

  std::ranges::for_each(faces, [](auto const& item) {
    printf("f %u %u %u\n", item.vertices[0] + 1, item.vertices[1] + 1, item.vertices[2] + 1);
  });

  return 0;
}