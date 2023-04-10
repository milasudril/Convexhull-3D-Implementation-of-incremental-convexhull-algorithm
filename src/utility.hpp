#include <vector>

struct Point3D {
  float x;
  float y;
  float z;
  float intensity;
  bool processed;
  Point3D() = default;
  Point3D(float _x, float _y, float _z):\
      x(_x), y(_y), z(_z), intensity(0), processed(false) {}
  Point3D(float _x, float _y, float _z, float _i):\
      x(_x), y(_y), z(_z), intensity(_i), processed(false) {}

  constexpr bool operator ==(const Point3D& pt) const
  {
      return x == pt.x && y == pt.y && z == pt.z;
  }

  constexpr bool operator !=(const Point3D& pt) const
  { return !(*this == pt); }

  float operator *(const Point3D& pt) const
  {
      return x * pt.x + y * pt.y + z * pt.z;
  }

  Point3D operator *(const float factor) const
  {
      return Point3D(x*factor, y*factor, z*factor);
  }

  Point3D operator /(const float factor) const
  {
      return Point3D(x/factor, y/factor, z/factor);
  }

  Point3D operator +(const Point3D& pt) const
  {
      return Point3D(x+pt.x, y+pt.y, z+pt.z);
  }

  Point3D operator -(const Point3D& pt) const
  {
      return Point3D(x-pt.x, y-pt.y, z-pt.z);
  }
};

inline bool Colinear(const Point3D& a, const Point3D& b, const Point3D& c)
{
  return ((c.z - a.z) * (b.y - a.y) -
          (b.z - a.z) * (c.y - a.y)) == 0 &&\
         ((b.z - a.z) * (c.x - a.x) -
          (b.x - a.x) * (c.z - a.z)) == 0 &&\
         ((b.x - a.x) * (c.y - a.y) -
          (b.y - a.y) * (c.x - a.x)) == 0;
}


struct PointHash
{
    size_t operator()(Point3D a) const
    {
    return std::hash<float>{}(a.x) ^ std::hash<float>{}(a.y) ^ std::hash<float>{}(a.z);
    }
};