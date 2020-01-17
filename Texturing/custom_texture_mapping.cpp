#include <pcl/surface/texture_mapping.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/features/normal_3d.h>
#include <blend2d.h>



int main(int argc, char** argv) {

	// ====== load geometry into a PolygonMesh
	pcl::PolygonMesh pmesh;
	std::string fname = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_decimated_0.900000/ptcloud_hd00000380_normals_cleaned.ply";
	pcl::io::loadPLYFile(fname, pmesh);

	// ====== transfer points / faces to a TextureMesh
	pcl::TextureMesh tmesh;
	tmesh.cloud = pmesh.cloud;
	std::vector< pcl::Vertices> tmesh_polygons;
	tmesh_polygons.resize(pmesh.polygons.size());

	for (size_t i = 0; i < pmesh.polygons.size(); ++i) {
		tmesh_polygons[i] = pmesh.polygons[i];
	}
	tmesh.tex_polygons.push_back(tmesh_polygons);

	// ====== create uv mapping
	pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
	pcl::TexMaterial tex_mat;
	tm.setTextureMaterials(tex_mat); // push null tex material so functions dont complain
	std::vector<std::string> tex_files;
	//tex_files.push_back("C:/Users/maxhu/Desktop/uvatlas_example/occluded.jpg");
	tex_files.push_back("C:/Users/maxhu/Desktop/uvatlas_example/custom_texture_mapping.bmp");
	tm.setTextureFiles(tex_files); // push dummy tex file too
	//tm.mapTexture2Mesh(tmesh);
	tm.mapTexture2MeshUV(tmesh);

	// ====== color in uv mapping
	BLImage img(480, 480, BL_FORMAT_PRGB32);
	BLContext ctx(img);
	ctx.setCompOp(BL_COMP_OP_SRC_OVER);

	// set black background
	ctx.setFillStyle(BLRgba32(0xFF000000));
	ctx.fillAll();

	// color in tris
	//ctx.setCompOp(BL_COMP_OP_PLUS); // this should work, except it doesn't bc im guessing the UV mapping overlaps triangles

	BLGradient linear_red(BLLinearGradientValues(0, 0, 0, 480));
	linear_red.addStop(0.0, BLRgba32(0xFF000000));
	linear_red.addStop(1.0, BLRgba32(0xFFFF0000));

	BLGradient linear_green(BLLinearGradientValues(0, 0, 480, 0));
	linear_green.addStop(0.0, BLRgba32(0xFF000000));
	linear_green.addStop(1.0, BLRgba32(0xFF00FF00));

	//ctx.setFillStyle(BLRgba32(0xFFFFFFFF)); // use to color all tris white
	std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> submesh = tmesh.tex_coordinates[0];
	for (int i = 0; i < submesh.size(); i+=3) {  // loop thru faces
		Eigen::Vector2f pt0 = submesh[i];
		Eigen::Vector2f pt1 = submesh[i+1];
		Eigen::Vector2f pt2 = submesh[i+2];

		// TODO: scale this better, does Blend2d have a pt struct?
		BLTriangle tri(pt0(0)*480, pt0(1)*480, pt1(0)*480, pt1(1)*480, pt2(0)*480, pt2(1)*480);

		ctx.setCompOp(BL_COMP_OP_SRC_OVER);
		ctx.setFillStyle(linear_red);
		ctx.fillTriangle(tri);

		ctx.setCompOp(BL_COMP_OP_PLUS);
		ctx.setFillStyle(linear_green);
		ctx.fillTriangle(tri);
	}


	// ====== write texture map to file
	BLImageCodec codec;
	codec.findByName("BMP");
	img.writeToFile("C:/Users/maxhu/Desktop/uvatlas_example/custom_texture_mapping.bmp", codec);

	// ====== compute normals for the mesh so that saveOBJFile() doesn't complain
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pmesh.cloud, *cloud);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	// Concatenate XYZ and normal fields
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	pcl::toPCLPointCloud2(*cloud_with_normals, tmesh.cloud);

	// ====== write TextureMesh to .obj
	pcl::io::saveOBJFile("C:/Users/maxhu/Desktop/uvatlas_example/custom_texture_mapping.obj", tmesh);
}