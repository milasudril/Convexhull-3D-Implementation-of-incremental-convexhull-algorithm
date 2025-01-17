#ifndef CONVHULL_MESHCLASSES_HPP
#define CONVHULL_MESHCLASSES_HPP

#include "./point.hpp"

#include <cstdint>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdlib>

namespace convhull
{
	class vertex_index
	{
	public:
		constexpr explicit vertex_index(uint32_t val):m_value{val}{}

		constexpr explicit vertex_index(point_3d const* ptr, point_3d const* base):
			m_value{static_cast<uint32_t>(ptr - base)}
		{}

		constexpr bool operator==(vertex_index const&) const = default;
		constexpr bool operator!=(vertex_index const&) const = default;

		constexpr auto value() const { return m_value; }

	private:
		uint32_t m_value;
	};

	constexpr auto operator+(point_3d* ptr, vertex_index vi)
	{
		return ptr + vi.value();
	}

	constexpr auto operator+(point_3d const* ptr, vertex_index vi)
	{
		return ptr + vi.value();
	}

	struct face
	{
		std::array<vertex_index, 3> vertices;
		bool visible{false};

		void flip()
		{ std::swap(vertices[0], vertices[2]); }
	};

	// A point is considered outside of a CCW face if the volume of tetrahedron
	// formed by the face and point is negative. Note that origin is set at p.
	inline auto signed_volume(point_3d const* vert_array, face const& f, point_3d p)
	{
		std::array<point_3d, 3> const vertices{
			*(vert_array + f.vertices[0]),
			*(vert_array + f.vertices[1]),
			*(vert_array + f.vertices[2])
		};

		auto const a = vertices[0] - p;
		auto const b = vertices[1] - p;
		auto const c = vertices[2] - p;

		return inner_product(a, cross(b, c));
	}

	inline auto make_oriented_face(point_3d const* vert_array,
		vertex_index a,
		vertex_index b,
		vertex_index c,
		point_3d ref)
	{
		face ret{a, b, c};
		if(signed_volume(vert_array, ret, ref) < 0.0f)
		{ ret.flip(); }

		return ret;
	}

	struct edge
	{
		constexpr explicit edge(vertex_index p1, vertex_index p2):
			endpoints{p1, p2}
		{}

		std::array<vertex_index, 2> endpoints;

		constexpr bool operator==(edge const&) const = default;
		constexpr bool operator!=(edge const&) const = default;
	};

	struct edge_data
	{
		explicit edge_data():
			adjface1{nullptr},
			adjface2{nullptr},
			to_be_removed{false}
		{}

		void link_face(face* face)
		{
			assert(adjface1 == nullptr || adjface2 == nullptr);

			if(adjface1 == nullptr)
			{ adjface1 = face; }
			else
			{ adjface2 = face; }
		}

		void erase(face const* face)
		{
			assert(adjface1 == face || adjface2 == face);

			if(adjface1 == face)
			{ adjface1 = nullptr; }
			else
			{ adjface2 = nullptr; }
		}

		face* adjface1;
		face* adjface2;
		bool to_be_removed;
	};

	// for face(a,b,c) and edge(a,c), return b
	inline auto find_inner_point(const face& f, const edge& e)
	{
		for(size_t i = 0; i != std::size(f.vertices); i++)
		{
			if(f.vertices[i] == e.endpoints[0]) continue;
			if(f.vertices[i] == e.endpoints[1]) continue;
			return f.vertices[i];
		}

		abort();
	}
}

#endif