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

//@ {"dependencies_extra":[{"ref":"./builder.o", "rel":"implementation"}]}

#ifndef CONVHULL_BUILDER_HPP
#define CONVHULL_BUILDER_HPP

#include "./point.hpp"
#include "./mesh_classes.hpp"

#include <vector>
#include <string>
#include <map>
#include <list>
#include <span>
#include <bit>
#include <cassert>
#include <functional>

namespace convhull
{
	namespace detail
	{
		struct edge_cmp
		{
			size_t operator()(edge const& a, edge const& b) const
			{
				auto const a_bits = std::bit_cast<size_t>(a);
				auto const b_bits = std::bit_cast<size_t>(b);
				return a_bits < b_bits;
			}
		};
	}

	using edge_map = std::map<edge, edge_data, detail::edge_cmp>;

	inline void create_and_link_edge(edge_map& edges, vertex_index p1, vertex_index p2, face& face)
	{
		auto const i = edges.insert(std::pair{edge{p1, p2}, edge_data{}});
		i.first->second.link_face(&face);
	}

	using face_list = std::list<face>;

	size_t mark_visible_faces(face_list& faces, point_3d const* points, point_3d cam_loc);

	void cleanup(edge_map& edges);

	void remove_hidden(face_list& faces);

	class builder
	{
		public:
			explicit builder(std::span<point_3d const> points):
				m_visited(std::size(points), 0)
			{ create(points); }

			face_list const& faces() const {return m_faces;}

		private:
			void insert_face(point_3d const* vert_array,
				vertex_index a,
				vertex_index b,
				vertex_index c,
				point_3d inner_pt);

			void insert_face(point_3d const* vert_array,
				std::pair<edge const, edge_data>& current_edge,
				vertex_index c,
				point_3d inner_pt);

			void create_seed(std::span<point_3d const> pointcloud);
			void try_insert(point_3d const* vert_array, std::reference_wrapper<point_3d const> p);
			void create(std::span<point_3d const> pointcloud);

			std::vector<int8_t> m_visited;
			face_list m_faces;
			edge_map m_edges;
	};
}

#endif