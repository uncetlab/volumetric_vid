#include <pcl/TextureMesh.h>
#include <blend2d.h>
#include "texturing.h"


void generateGradientTexture(const std::string &file_name, std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> &tex_coords) {
	// create image for drawing on
	BLImage img(480, 480, BL_FORMAT_PRGB32);
	BLContext ctx(img);
	ctx.setCompOp(BL_COMP_OP_SRC_OVER);

	// set black background
	ctx.setFillStyle(BLRgba32(0xFF000000));
	ctx.fillAll();

	// create gradients
	ctx.setCompOp(BL_COMP_OP_PLUS); // this should work, except it doesn't bc im guessing the UV mapping overlaps triangles

	BLGradient linear_red(BLLinearGradientValues(0, 0, 0, 480));
	linear_red.addStop(0.0, BLRgba32(0xFF000000));
	linear_red.addStop(1.0, BLRgba32(0xFFFF0000));

	BLGradient linear_green(BLLinearGradientValues(0, 0, 480, 0));
	linear_green.addStop(0.0, BLRgba32(0xFF000000));
	linear_green.addStop(1.0, BLRgba32(0xFF00FF00));

	//ctx.setFillStyle(BLRgba32(0xFFFFFFFF)); // use to color all tris white

	// color in tris
	for (int i = 0; i < tex_coords.size(); i += 3) {  // loop thru faces
		Eigen::Vector2f pt0 = tex_coords[i];
		Eigen::Vector2f pt1 = tex_coords[i + 1];
		Eigen::Vector2f pt2 = tex_coords[i + 2];

		// TODO: scale this better, does Blend2d have a pt struct?
		BLTriangle tri(pt0(0) * 480, pt0(1) * 480, pt1(0) * 480, pt1(1) * 480, pt2(0) * 480, pt2(1) * 480);

		//ctx.setCompOp(BL_COMP_OP_SRC_OVER);
		ctx.setFillStyle(linear_red);
		ctx.fillTriangle(tri);

		//ctx.setCompOp(BL_COMP_OP_PLUS);
		ctx.setFillStyle(linear_green);
		ctx.fillTriangle(tri);
	}

	// write texture map to file
	BLImageCodec codec;
	codec.findByName("BMP");
	img.writeToFile(file_name.c_str(), codec);
}