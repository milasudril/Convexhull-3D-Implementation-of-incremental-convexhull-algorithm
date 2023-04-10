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
	}
}

#endif