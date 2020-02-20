#include <string>
#include <volcap/io/io.h>
#include <volcap/io/io_fbx.h>
#include <pcl/io/obj_io.h>
#include <pcl/TextureMesh.h>

int main(int argc, char** argv) {

	////==> single example
	//pcl::TextureMesh tmesh;
	//pcl::io::loadOBJFile("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_texture_mapped/ptcloud_hd00000380_normals_cleaned.obj", tmesh);
	//volcap::io::fbxFromTextureMesh(tmesh, "mesh_name", "C:/Users/maxhu/Desktop/uvatlas_example/fbx_sdk/ptcloud_hd00000380_normals_cleaned.fbx");

	//==> convert a dir of .obj files into .fbx
	std::vector<pcl::TextureMeshPtr> meshes;
	std::vector<std::string> mesh_filenames;
	//std::string input_dir = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient";
	//std::string input_dir = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient/pngs";
	std::string input_dir = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_texture_mapped";
	volcap::io::load_meshes_from_dir(input_dir, meshes, mesh_filenames);

	std::string output_dir = input_dir + "/fbx/";

	for (int i = 0; i < meshes.size(); i++) {
		volcap::io::fbxFromTextureMesh(*meshes[i], mesh_filenames[i], output_dir + mesh_filenames[i]);
	}
}
