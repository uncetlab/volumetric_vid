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

	///********* export to a single file with timesamples *********/
	volcap::io::UsdExporter exporter(output_dir + "/usd_seq.usda");
	//volcap::io::UsdExporter exporter(output_dir + "/usd_seq.usdc");  // can also export to .usdc, but usdzip does this for us
	exporter.usdFromTextureMeshes(meshes, mesh_filenames);

	// manually rotate this sequence so the up axis is y+ and dude is facing forward direction (z+)
	exporter.rotateXYZ(180, 45, 0);

	exporter.save();

	/********* export to multiple files with no timesamples *********/
	for (int i = 0; i < meshes.size(); i++) {
		volcap::io::UsdExporter exporter(output_dir + "/" + mesh_filenames[i] + ".usda");
		exporter.usdFromTextureMesh(meshes[i], mesh_filenames[i]);

		// manually rotate this sequence so the up axis is y+ and dude is facing forward direction (z+)
		exporter.rotateXYZ(180, 45, 0);

		exporter.save();
	}
}
