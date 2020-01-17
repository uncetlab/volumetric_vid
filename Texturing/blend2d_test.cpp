#include <blend2d.h>

// example from blend2d getting started
//int main(int argc, char* argv[]) {
//	BLImage img(480, 480, BL_FORMAT_PRGB32);
//
//	// Attach a rendering context into `img`.
//	BLContext ctx(img);
//
//	// Clear the image.
//	ctx.setCompOp(BL_COMP_OP_SRC_COPY);
//	ctx.fillAll();
//
//	// Fill some path.
//	BLPath path;
//	path.moveTo(26, 31);
//	path.cubicTo(642, 132, 587, -136, 25, 464);
//	path.cubicTo(882, 404, 144, 267, 27, 31);
//
//	ctx.setCompOp(BL_COMP_OP_SRC_OVER);
//	ctx.setFillStyle(BLRgba32(0xFFFFFFFF));
//	ctx.fillPath(path);
//
//	// Detach the rendering context from `img`.
//	ctx.end();
//
//	// Let's use some built-in codecs provided by Blend2D.
//	BLImageCodec codec;
//	codec.findByName("BMP");
//	img.writeToFile("C:/Users/maxhu/Desktop/uvatlas_example/bl-getting-started-1.bmp", codec);
//
//	return 0;
//}

// try out loading jpeg into memory, copying texture to another file
int main(int argc, char* argv[]) {
	BLImage img(480, 480, BL_FORMAT_PRGB32);
	BLContext ctx(img);

	BLImage texture;
	BLResult err = texture.readFromFile("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinectImgs/50_02/50_02_00000001.jpg");

	// Basic error handling is necessary as we need some IO.
	if (err) {
		printf("Failed to load a texture (err=%u)\n", err);
		return 1;
	}

	ctx.setCompOp(BL_COMP_OP_SRC_OVER);
	ctx.setFillStyle(BLPattern(texture));

	BLTriangle tri(240, 0, 0, 240, 480, 240);
	ctx.fillTriangle(tri);

	BLImageCodec codec;
	codec.findByName("BMP");
	err = img.writeToFile("C:/Users/maxhu/Desktop/uvatlas_example/copy_tri.bmp", codec);
	if (err) {
		printf("Encoder error (err=%u)\n", err);
		return 1;
	}

	codec.findByName("JPEG");
	err = img.writeToFile("C:/Users/maxhu/Desktop/uvatlas_example/copy_tri.jpeg", codec);
	if (err) {
		printf("Encoder error (err=%u)\n", err);
		return 1;
	}

	codec.findByName("PNG");
	err = img.writeToFile("C:/Users/maxhu/Desktop/uvatlas_example/copy_tri.png", codec);
	if (err) {
		printf("Encoder error (err=%u)\n", err);
		return 1;
	}

	return 0;
}