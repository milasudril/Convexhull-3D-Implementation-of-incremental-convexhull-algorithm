//@ {"target":{"name":"plain_demo.o"}}

#include "./convexhull.hpp"
#include <vector>
#include <array>
#include <cassert>
#include <algorithm>
#include <optional>

struct face_indirect
{
  std::array<uint32_t, 3> vertices;
};

std::vector<Point3D> load_points(FILE* stream)
{
  std::vector<Point3D> ret;
  enum class state{newline, skipline, vertex_begin, coords};
  auto current_state = state::newline;

  std::string buffer;
  size_t fieldcount = 0;
  Point3D point;

  while(true)
  {
    auto const ch_in = getc(stream);
    if(ch_in == EOF)
    {
      if(fieldcount != 0)
      { ret.push_back(point); }

      return ret;
    }

    switch(current_state)
    {
      case state::newline:
        switch(ch_in)
        {
          case '#':
            current_state = state::skipline;
            break;
          case 'v':
            current_state = state::vertex_begin;
            break;
          case 'f':
            current_state = state::skipline;
            break;
          case 'l':
            current_state = state::skipline;
            break;
          default:
            throw std::runtime_error{"Invalid line"};
        }
        break;
      case state::skipline:
        if(ch_in == '\n')
        { current_state = state::newline; }
        break;

      case state::vertex_begin:
        switch(ch_in)
        {
          case ' ':
            current_state = state::coords;
            break;
          case 't':
            current_state = state::skipline;
            break;
          case 'p':
            current_state = state::skipline;
            break;
          case 'n':
            current_state = state::skipline;
            break;
          default:
            throw std::runtime_error{"Invalid line"};
        }
        break;

      case state::coords:
        switch(ch_in)
        {
          case ' ':
            switch(fieldcount)
            {
              case 0:
                point.x = std::stof(buffer);
                break;
              case 1:
                point.y = std::stof(buffer);
                break;
              case 2:
                point.z = std::stof(buffer);
                break;
            }
            buffer.clear();
            ++fieldcount;
            break;

          case '\n':
            switch(fieldcount)
            {
              case 0:
                point.x = std::stof(buffer);
                break;
              case 1:
                point.y = std::stof(buffer);
                break;
              case 2:
                point.z = std::stof(buffer);
                break;
            }
            buffer.clear();
            fieldcount = 0;
            ret.push_back(point);
            current_state = state::newline;
            break;

          default:
            buffer += static_cast<char>(ch_in);
        }
        break;
    }
  };
}

int main()
{
  ConvexHull C(load_points(stdin));

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