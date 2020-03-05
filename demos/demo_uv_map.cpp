#include <volcap/texture/uv_map.h>
#include <volcap/io/io.h>
#include <boost/filesystem.hpp>
#include <pcl/io/obj_io.h>

// get the project directory from preprocessor set in the top-level CMakeLists.txt
const std::string PROJECT_DIR = _PROJECT_DIR;

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
	boost::filesystem::path input_dir(PROJECT_DIR + "/demos/demo_output/03_mesh_decimated/decimated_0.800000");
	boost::filesystem::path out_dir(PROJECT_DIR + "/demos/demo_output/04_mesh_uv-mapped");
	boost::filesystem::create_directory(out_dir);

	std::vector<pcl::PolygonMeshPtr> p_meshes;
	std::vector<std::string> mesh_paths;
	volcap::io::load_meshes_from_dir(input_dir.string(), p_meshes, mesh_paths);

	for (int idx_mesh = 0; idx_mesh < p_meshes.size(); idx_mesh++) {
	//for (int idx_mesh = 0; idx_mesh < 1; idx_mesh++) {

		pcl::PolygonMesh pmesh = *p_meshes[idx_mesh];
		pcl::TextureMesh tmesh;  // use TextureMesh so we can store texture coordinates
		volcap::texture::generateUVMapping(pmesh, tmesh);

		////ttestst
		//pcl::TexMaterial mesh_material;
		//mesh_material.tex_name = "material_0";
		//tmesh.tex_materials.push_back(mesh_material);

		boost::filesystem::path file_path(mesh_paths[idx_mesh]);
		std::string file_name = file_path.stem().string();

		//==> save to obj
		boost::filesystem::path out_file = out_dir / (file_name + ".obj");
		//out_file.make_preferred();
		std::string out_file_str = out_file.string();
		std::replace(out_file_str.begin(), out_file_str.end(), '\\', '/');
		pcl::io::saveOBJFile(out_file_str, tmesh);  // does not detect backslashes properly when saving .mtl

		printf("saved obj to: %s\n", out_file_str.c_str());
	}

}
