#include <volcap/texture/uv_map.h>
#include <volcap/io/io.h>
#include <boost/filesystem.hpp>
#include <pcl/io/obj_io.h>


//void gradient_demo() {
//	// generating texture based on coords loaded from .obj (so we dont have to run UVAtlas again)
//	// load TextureMesh from .obj
//
//	//pcl::TextureMeshPtr texture_mesh(boost::make_shared<pcl::TextureMesh>());
//	pcl::TextureMesh texture_mesh;
//	pcl::io::loadOBJFile("C:/Users/maxhu/Desktop/uvatlas_example/test_panoptic.obj", texture_mesh);
//	volcap::texture::generateGradientTexture("C:/Users/maxhu/Desktop/uvatlas_example/test_panoptic.bmp", texture_mesh.tex_coordinates[0]);
//}

/*
 * Demonstrates Microsoft UVAtlas being used to create a UV mapping for a mesh obtained after SPSR reconstruction
 */
int main(int argc, char** argv)
{
	std::vector<pcl::PolygonMeshPtr> p_meshes;
	std::vector<std::string> mesh_ids;
	std::string dir_name = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_decimated_0.900000";

	volcap::io::load_meshes_from_dir(dir_name, p_meshes, mesh_ids);

	//std::string out_dir = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient/";
	//std::string out_dir = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient/pngs/";
	std::string out_dir = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/uvatlas_mapped/";

	//for (int idx_mesh = 0; idx_mesh < p_meshes.size(); idx_mesh++) {
	for (int idx_mesh = 0; idx_mesh < 2; idx_mesh++) {

		pcl::PolygonMesh pmesh = *p_meshes[idx_mesh];
		pcl::TextureMesh tmesh;
		volcap::texture::generateUVMapping(pmesh, tmesh);

		boost::filesystem::path file_path(mesh_ids[idx_mesh]);
		std::string file_name = file_path.stem().string();

		//==> save to obj
		std::string obj_path = out_dir + file_name + ".obj";  // path must be declared with forward slashes to work correctly
		pcl::io::saveOBJFile(obj_path, tmesh);

		printf("saved obj to: %s\n", obj_path.c_str());
	}

}
