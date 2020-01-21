#include <pcl/surface/texture_mapping.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/features/normal_3d.h>
#include <blend2d.h>
#include "texturing.h"


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

	// ====== generate texture with uv mapping
	generateGradientTexture("C:/Users/maxhu/Desktop/uvatlas_example/custom_texture_mapping.bmp", tmesh.tex_coordinates[0]);

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