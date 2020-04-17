#include <volcap/texture/uv_map.h>
#include <volcap/io/io.h>
#include <boost/filesystem.hpp>
#include <pcl/io/obj_io.h>
#include <chrono>

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

// test out different levels [0.0, 0.2, 0.4, 0.6, 0.8, 1.0] of UVAtlas' stretch factor
void test_stretch(pcl::PolygonMeshPtr pmesh, std::string mesh_path, boost::filesystem::path out_dir) {

	for (float maxStretch = 0.0f; maxStretch < 1.1f; maxStretch += 0.2) {
		pcl::TextureMesh tmesh;  // use TextureMesh so we can store texture coordinates

		// record how long generating uv map takes for each stretch factor
		auto startTime = std::chrono::system_clock::now();
		volcap::texture::generateUVMapping(*pmesh, tmesh, maxStretch);
		std::chrono::duration<double> duration = std::chrono::system_clock::now() - startTime;

		//==> save to obj
		boost::filesystem::path file_path(mesh_path);
		std::string file_name = file_path.stem().string() + "_" + std::to_string(maxStretch); // todo: use string formatting (req c++20)

		boost::filesystem::path out_file = out_dir / (file_name + ".obj");
		std::string out_file_str = out_file.string();

		// saveOBJFile() does not detect backslashes properly when saving .mtl,
		// we must replace backslashes with forward slashes
		std::replace(out_file_str.begin(), out_file_str.end(), '\\', '/');

		pcl::io::saveOBJFile(out_file_str, tmesh);

		printf("[maxStretch %f] finished in [%f] sec\n", maxStretch, duration.count());
		printf("saved obj to: %s\n", out_file_str.c_str());
	}

}

void run_demo_sequence(
	std::vector<pcl::PolygonMeshPtr> &p_meshes,
	std::vector<std::string> &mesh_paths,
	boost::filesystem::path out_dir,
	float maxStretch
) {

	for (int idx_mesh = 0; idx_mesh < p_meshes.size(); idx_mesh++) {

		pcl::PolygonMesh pmesh = *p_meshes[idx_mesh];
		pcl::TextureMesh tmesh;  // use TextureMesh so we can store texture coordinates
		volcap::texture::generateUVMapping(pmesh, tmesh, maxStretch);

		////ttestst
		//pcl::TexMaterial mesh_material;
		//mesh_material.tex_name = "material_0";
		//tmesh.tex_materials.push_back(mesh_material);

		boost::filesystem::path file_path(mesh_paths[idx_mesh]);
		std::string file_name = file_path.stem().string();

		//==> save to obj
		boost::filesystem::path out_file = out_dir / (file_name + ".obj");
		std::string out_file_str = out_file.string();

		// saveOBJFile() does not detect backslashes properly when saving .mtl,
		// we must replace backslashes with forward slashes
		std::replace(out_file_str.begin(), out_file_str.end(), '\\', '/');

		pcl::io::saveOBJFile(out_file_str, tmesh);
		printf("saved obj to: %s\n", out_file_str.c_str());
	}
}

/*
 * Demonstrates Microsoft UVAtlas being used to create a UV mapping for a mesh obtained after SPSR reconstruction
 */
int main(int argc, char** argv)
{

	float maxStretch = 0.6f;
	boost::filesystem::path input_dir(PROJECT_DIR + "/demos/demo_output/03_mesh_decimated/decimated_0.800000");
	//boost::filesystem::path out_dir(PROJECT_DIR + "/demos/demo_output/04_mesh_uv-mapped/stretch_" + std::to_string(maxStretch));
	boost::filesystem::path out_dir(PROJECT_DIR + "/demos/demo_output/04_mesh_uv-mapped");
	boost::filesystem::create_directory(out_dir);

	std::vector<pcl::PolygonMeshPtr> p_meshes;
	std::vector<std::string> mesh_paths;
	volcap::io::load_meshes_from_dir(input_dir.string(), p_meshes, mesh_paths);

	run_demo_sequence(p_meshes, mesh_paths, out_dir.string(), maxStretch);

	//boost::filesystem::path out_dir_test(PROJECT_DIR + "/demos/demo_output/04_mesh_uv-mapped/stretch_test");
	//boost::filesystem::create_directory(out_dir_test);
	//test_stretch(p_meshes[0], mesh_paths[0], out_dir_test.string());
}
