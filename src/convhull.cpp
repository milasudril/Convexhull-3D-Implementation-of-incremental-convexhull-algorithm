//@ {"target":{"name":"convhull.o"}}

#include "./builder.hpp"

#include <vector>
#include <cassert>
#include <algorithm>

std::vector<convhull::point_3d> load_points(FILE* stream)
{
	std::vector<convhull::point_3d> ret;
	enum class state{newline, skipline, vertex_begin, coords};
	auto current_state = state::newline;

	std::string buffer;
	size_t fieldcount = 0;
	std::array<float, 3> point;

	while(true)
	{
		auto const ch_in = getc(stream);
		if(ch_in == EOF)
		{
			if(fieldcount != 0)
			{ ret.push_back(convhull::point_3d{point[0], point[1], point[2]}); }

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
								point[0] = std::stof(buffer);
								break;
							case 1:
								point[1] = std::stof(buffer);
								break;
							case 2:
								point[2] = std::stof(buffer);
								break;
						}
						buffer.clear();
						++fieldcount;
						break;

					case '\n':
						switch(fieldcount)
						{
							case 0:
								point[0] =  std::stof(buffer);
								break;
							case 1:
								point[1] =  std::stof(buffer);
								break;
							case 2:
								point[2] =  std::stof(buffer);
								break;
						}
						buffer.clear();
						fieldcount = 0;
						ret.push_back(convhull::point_3d{point[0], point[1], point[2]});
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
	auto const points = load_points(stdin);

	convhull::builder builder{points};

	std::ranges::for_each(points, [](auto const& item){
		printf("v %.8g %.8g %.8g\n", item[0], item[1], item[2]);
	});

	std::ranges::for_each(builder.faces(), [](auto const& item) {
		printf("f %u %u %u\n", item.vertices[0].value() + 1, item.vertices[1].value() + 1, item.vertices[2].value() + 1);
	});

	return 0;
}