#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/io/ply_io.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <volcap/surface/spsr.h>

// get the project directory from preprocessor set in the top-level CMakeLists.txt
const std::string PROJECT_DIR = _PROJECT_DIR;

// Performs SPSR on a short sequence of (.ply) point clouds
int main(int argc, char** argv)
{
	boost::filesystem::path input_dir(PROJECT_DIR + "/demos/demo_output/01_cloud_mls");
	boost::filesystem::path output_dir_spsr(PROJECT_DIR + "/demos/demo_output/02_mesh_spsr");
	boost::filesystem::path output_dir_decimated(PROJECT_DIR + "/demos/demo_output/03_mesh_decimated");
	boost::filesystem::create_directories(output_dir_spsr);
	boost::filesystem::create_directories(output_dir_decimated);
	
	// loop through clean point clouds
	boost::filesystem::directory_iterator it{ input_dir };
	while (it != boost::filesystem::directory_iterator{}) {

		// Get input paths
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path in_p = entry.path();

		if (!(in_p.has_extension() && in_p.extension() == ".ply"))  // skip non .ply files
			continue;

		//std::cout << entry << '\n' << in_p << '\n' << out_p << '\n';
		printf("file to be loaded: %s\n", in_p.string().c_str());

		// Read in point cloud into cloud_xyzrgbnormals
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgbnormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::io::loadPLYFile(in_p.string(), *cloud_xyzrgbnormals);

		// Compute SPSR
		pcl::PolygonMesh::Ptr spsr_mesh(boost::make_shared<pcl::PolygonMesh>());
		volcap::surface::compute_mesh<pcl::PointXYZRGBNormal>(cloud_xyzrgbnormals, *spsr_mesh);

		// Decimate mesh
		float percent_faces_removed = 0.8;

		pcl::PolygonMesh decimated_mesh;
		pcl::MeshQuadricDecimationVTK decimator;
		decimator.setTargetReductionFactor(percent_faces_removed);
		decimator.setInputMesh(spsr_mesh);
		decimator.process(decimated_mesh);

		// Get / create output paths
		std::string decimated_folder_str = "decimated_";
		decimated_folder_str += std::to_string(percent_faces_removed);

		boost::filesystem::path out_spsr_file = output_dir_spsr / in_p.filename();
		boost::filesystem::path out_decimated_file = output_dir_decimated / decimated_folder_str / in_p.filename();
		boost::filesystem::create_directory(out_decimated_file.parent_path());

		// Save meshes to ply
		pcl::io::savePLYFile(out_spsr_file.string(), *spsr_mesh);
		pcl::io::savePLYFile(out_decimated_file.string(), decimated_mesh);
	}

	return (0);
}