#ifndef CONVHULL_POINT_HPP
#define CONVHULL_POINT_HPP

namespace convhull
{
    struct point_3d
    {
        float x;
        float y;
        float z;
        float intensity;

        point_3d() = default;
        explicit point_3d(float _x, float _y, float _z): x(_x), y(_y), z(_z), intensity(0.0f) {}
    };

    inline bool colinear(point_3d a, point_3d b, point_3d c)
    {
        return ((c.z - a.z) * (b.y - a.y) -
                (b.z - a.z) * (c.y - a.y)) == 0.0f &&
                ((b.z - a.z) * (c.x - a.x) -
                (b.x - a.x) * (c.z - a.z)) == 0.0f &&
                ((b.x - a.x) * (c.y - a.y) -
                (b.y - a.y) * (c.x - a.x)) == 0.0f;
    }
}

#endif