#include <blend2d.h>
#include <Eigen/Dense>

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

//// try out loading jpeg into memory, copying texture to another file
//int main(int argc, char* argv[]) {
//	BLImage img(480, 480, BL_FORMAT_PRGB32);
//	BLContext ctx(img);
//
//	BLImage texture;
//	BLResult err = texture.readFromFile("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinectImgs/50_02/50_02_00000001.jpg");
//
//	// Basic error handling is necessary as we need some IO.
//	if (err) {
//		printf("Failed to load a texture (err=%u)\n", err);
//		return 1;
//	}
//
//	ctx.setCompOp(BL_COMP_OP_SRC_OVER);
//	ctx.setFillStyle(BLPattern(texture));
//
//	BLTriangle tri(240, 0, 0, 240, 480, 240);
//	ctx.fillTriangle(tri);
//
//	BLImageCodec codec;
//	codec.findByName("BMP");
//	err = img.writeToFile("C:/Users/maxhu/Desktop/uvatlas_example/copy_tri.bmp", codec);
//	if (err) {
//		printf("Encoder error (err=%u)\n", err);
//		return 1;
//	}
//
//	codec.findByName("JPEG");
//	err = img.writeToFile("C:/Users/maxhu/Desktop/uvatlas_example/copy_tri.jpeg", codec);
//	if (err) {
//		printf("Encoder error (err=%u)\n", err);
//		return 1;
//	}
//
//	codec.findByName("PNG");
//	err = img.writeToFile("C:/Users/maxhu/Desktop/uvatlas_example/copy_tri.png", codec);
//	if (err) {
//		printf("Encoder error (err=%u)\n", err);
//		return 1;
//	}
//
//	return 0;
//}

//// texture rotate test
//int main(int argc, char* argv[]) {
//	BLImage img(480, 480, BL_FORMAT_PRGB32);
//	BLContext ctx(img);
//
//	ctx.setCompOp(BL_COMP_OP_SRC_COPY);
//	ctx.fillAll();
//
//	// Read an image from file.
//	BLImage texture;
//	BLResult err = texture.readFromFile("C:/Users/maxhu/Desktop/uvatlas_example/lenna_test_image.png");
//
//	// Basic error handling is necessary as we need some IO.
//	if (err) {
//		printf("Failed to load a texture (err=%u)\n", err);
//		return 1;
//	}
//
//	// Create a pattern and use it to fill a rounded-rect.
//	BLPattern pattern(texture);
//	pattern.rotate(0.785398, 240.0, 240.0);
//
//	ctx.setCompOp(BL_COMP_OP_SRC_OVER);
//	ctx.setFillStyle(pattern);
//	ctx.fillRoundRect(40.0, 40.0, 400.0, 400.0, 45.5);
//
//	ctx.end();
//
//	BLImageCodec codec;
//	codec.findByName("BMP");
//	img.writeToFile("C:/Users/maxhu/Desktop/uvatlas_example/blend2d_tests/texture_transform_test.bmp", codec);
//
//	return 0;
//}

// copy tri test
int main(int argc, char* argv[]) {
	BLImage img(512, 512, BL_FORMAT_PRGB32);
	BLContext ctx(img);

	ctx.setCompOp(BL_COMP_OP_SRC_COPY);
	ctx.fillAll();

	// Read an image from file.
	BLImage texture;
	BLResult err = texture.readFromFile("C:/Users/maxhu/Desktop/uvatlas_example/lenna_test_image.png");

	// Basic error handling is necessary as we need some IO.
	if (err) {
		printf("Failed to load a texture (err=%u)\n", err);
		return 1;
	}

	// Determine tri transformation

	Eigen::Vector2f img_pt0(400, 500);
	Eigen::Vector2f img_pt1(500, 500);
	Eigen::Vector2f img_pt2(500, 450);

	Eigen::Vector2f uv_pt0(12, 100);
	Eigen::Vector2f uv_pt1(256, 256);
	Eigen::Vector2f uv_pt2(100, 12);

	Eigen::Matrix3f img_pts;	// starting points
	img_pts <<	img_pt0.x(), img_pt1.x(), img_pt2.x(),
				img_pt0.y(), img_pt1.y(), img_pt2.y(),
				1, 1, 1;

	Eigen::Matrix3f uv_pts;		// points after transformation is applied
	uv_pts <<	uv_pt0.x(), uv_pt1.x(), uv_pt2.x(),
				uv_pt0.y(), uv_pt1.y(), uv_pt2.y(),
				1, 1, 1;

	Eigen::Matrix3f inv = img_pts.inverse().eval();
	Eigen::Matrix3f transformation = uv_pts * inv;

	// Create a pattern and use it to fill a rounded-rect.
	BLPattern pattern(texture);
	BLMatrix2D bl_matrix(
		transformation(0, 0), transformation(1, 0),
		transformation(0, 1), transformation(1, 1),
		transformation(0, 2), transformation(1, 2)
	);

	ctx.setCompOp(BL_COMP_OP_SRC_OVER);
	ctx.setFillStyle(pattern);
	ctx.fillTriangle(
		img_pt0.x(), img_pt0.y(),
		img_pt1.x(), img_pt1.y(),
		img_pt2.x(), img_pt2.y()
	);

	pattern.transform(bl_matrix);
	ctx.setFillStyle(pattern);
	ctx.fillTriangle(
		uv_pt0.x(), uv_pt0.y(),
		uv_pt1.x(), uv_pt1.y(),
		uv_pt2.x(), uv_pt2.y()
	);

	ctx.end();

	BLImageCodec codec;
	codec.findByName("BMP");
	img.writeToFile("C:/Users/maxhu/Desktop/uvatlas_example/blend2d_tests/tri_transform_test.bmp", codec);

	return 0;
}