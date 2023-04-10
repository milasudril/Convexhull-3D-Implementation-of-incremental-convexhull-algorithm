## Incremental 3D-Convexhull algorithm
This repository contains an C++ implementation of 3D ConvexHull algorithm from the
book [Computational Geometry in C by O'Rourke](http://crtl-i.com/PDF/comp_c.pdf). This code is
written using standard containers, together with GeoSIMD. The latter is used to store points.

[![Example video](https://media.giphy.com/media/hsV1GgRby1M4kDbAgm/giphy.gif)](https://youtu.be/DDgGc7_fEyU)

## Build and Run (text output only)

```bash
make
__targets_rel/src/convhull < data/bunny.obj > bunny_convhull.obj
```

## Example Usage

```c++
// points can be converted into a
//
// span<convhull::point_3d const>;

convhull::builder builder{points};

std::ranges::for_each(points, [](auto const& item){
	printf("v %.8g %.8g %.8g\n", item[0], item[1], item[2]);
});

std::ranges::for_each(builder.faces(), [](auto const& item) {
	printf("f %u %u %u\n",
		item.vertices[0].value() + 1,
		item.vertices[1].value() + 1,
		item.vertices[2].value() + 1);
});
```

## License

MIT License
