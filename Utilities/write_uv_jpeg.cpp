#include <boost/gil.hpp>
//#include <boost/gil/extension/io/jpeg.hpp>

const unsigned WIDTH = 512;
const unsigned HEIGHT = 512;

int main()
{
	// Raw data.
	unsigned char r[WIDTH * HEIGHT];  // red
	unsigned char g[WIDTH * HEIGHT];  // green
	unsigned char b[WIDTH * HEIGHT];  // blue

	boost::gil::rgb8c_planar_view_t view =
		boost::gil::planar_rgb_view(WIDTH, HEIGHT, r, g, b, WIDTH);

	boost::gil::jpeg_write_view("out.jpg", view);

	return 0;
}
