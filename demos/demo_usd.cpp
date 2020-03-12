#include <string>
#include <volcap/io/io.h>
#include <volcap/io/io_usd.h>
#include <pcl/TextureMesh.h>
#include <boost/filesystem.hpp>

// get the project directory from preprocessor set in the top-level CMakeLists.txt
const std::string PROJECT_DIR = _PROJECT_DIR;

int main(int argc, char** argv) {

	//==> convert a dir of .obj files into .usda
	std::string input_dir = PROJECT_DIR + "/demos/demo_output/05_mesh_textured";
	std::string output_dir = PROJECT_DIR + "/demos/demo_output/07_mesh_usd";
	boost::filesystem::create_directories(output_dir);

	std::vector<pcl::TextureMeshPtr> meshes;
	std::vector<std::string> mesh_filenames;
	volcap::io::load_meshes_from_dir(input_dir, meshes, mesh_filenames);

	//volcap::io::usdFromTextureMeshes(meshes, mesh_filenames, output_dir + "/usd_seq.usda");

	volcap::io::UsdExporter exporter(output_dir + "/usd_seq.usda");
	exporter.usdFromTextureMeshes(meshes, mesh_filenames);
	exporter.save();
}
