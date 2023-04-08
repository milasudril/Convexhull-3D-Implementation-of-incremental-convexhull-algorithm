//@ {"target":{"name":"plain_demo.o"}}

#include "./convexhull.hpp"
#include <vector>

int main()
{
  std::vector<Point3D> tests = \
    {{3,0,0}, {0,3,0}, {0,0,3}, {3,3,3}};
  ConvexHull C(tests);

  auto const& verts = C.GetVertices();

  for(size_t k = 0; k != std::size(verts); ++k)
  {
    printf("%.8g %.8g %.8g\n", verts[k].x, verts[k].y, verts[k].z);
  }

  return 0;
}