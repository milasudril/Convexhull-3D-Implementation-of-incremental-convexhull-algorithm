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

  bool operator ==(const Point3D& pt) const
  {
      return x == pt.x && y == pt.y && z == pt.z;
  }

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

using PointStack = std::vector<Point3D>;
