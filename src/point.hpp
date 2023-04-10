#ifndef CONVHULL_POINT_HPP
#define CONVHULL_POINT_HPP

#include <geosimd/euclidian_space.hpp>

namespace convhull
{
	using point_3d = geosimd::euclidian_space<float, 3>::location;

	inline bool colinear(point_3d a, point_3d b, point_3d c)
	{
		auto const d1 = b - a;
		auto const d2 = c - a;
		return inner_product(d1, d2) == 0.0f;

#if 0
		return ((c.z - a.z) * (b.y - a.y) -
				(b.z - a.z) * (c.y - a.y)) == 0.0f &&
				((b.z - a.z) * (c.x - a.x) -
				(b.x - a.x) * (c.z - a.z)) == 0.0f &&
				((b.x - a.x) * (c.y - a.y) -
				(b.y - a.y) * (c.x - a.x)) == 0.0f;

#endif
	}

#if 0
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

	}
#endif
}

#endif