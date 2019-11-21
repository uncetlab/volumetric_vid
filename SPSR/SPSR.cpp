#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/io/ply_io.h>
#include <boost/filesystem.hpp>
#include <iostream>


void compute_mesh(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &input, pcl::PolygonMesh &output, int depth, int solver_divide, int iso_divide, float point_weight) {
	pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
	poisson.setDepth(depth);
	poisson.setSolverDivide(solver_divide);
	poisson.setIsoDivide(iso_divide);
	poisson.setPointWeight(point_weight);
	poisson.setInputCloud(input);
	poisson.reconstruct(output);
}

// Performs SPSR on a short sequence of point clouds
int main(int argc, char** argv)
{
	int default_depth = 8;
	int default_solver_divide = 8;
	int default_iso_divide = 8;
	float default_point_weight = 4.0f;
	
	// loop through clean point clouds
	boost::filesystem::path input_dir("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/normals_cleaned");
	boost::filesystem::directory_iterator it{ input_dir };
	while (it != boost::filesystem::directory_iterator{}) {

		// Get input paths
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path in_p = entry.path();

		//std::cout << entry << '\n' << in_p << '\n' << out_p << '\n';

		// Read in point cloud into cloud_xyzrgbnormals
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgbnormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::io::loadPLYFile(in_p.string(), *cloud_xyzrgbnormals);

		// Compute reconstructed mesh
		pcl::PolygonMesh::Ptr output(boost::make_shared<pcl::PolygonMesh>());
		compute_mesh(cloud_xyzrgbnormals, *output, default_depth, default_solver_divide, default_iso_divide, default_point_weight);

		// Decimate mesh
		float percent_faces_removed = 0.9;

		pcl::PolygonMesh decimated_output;
		pcl::MeshQuadricDecimationVTK decimator;
		decimator.setTargetReductionFactor(percent_faces_removed);
		decimator.setInputMesh(output);
		decimator.process(decimated_output);

		// Get / create output paths
		std::string decimated_folder_str = "decimated_";
		decimated_folder_str += std::to_string(percent_faces_removed);

		boost::filesystem::path out_p = in_p.parent_path().parent_path() / "pcl_mesh" / "full" / in_p.filename();
		boost::filesystem::create_directory(out_p.parent_path());
		boost::filesystem::path decimated_out_p = in_p.parent_path().parent_path() / "pcl_mesh" / decimated_folder_str / in_p.filename();
		boost::filesystem::create_directory(decimated_out_p.parent_path());

		// Save reconstructed mesh to ply
		pcl::io::savePLYFile(out_p.string(), *output);
		pcl::io::savePLYFile(decimated_out_p.string(), decimated_output);
	}

	return (0);
}