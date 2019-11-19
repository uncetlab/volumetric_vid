#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>
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

		// Get input / output paths
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path in_p = entry.path();
		boost::filesystem::path out_p = in_p.parent_path().parent_path() / "pcl_mesh" / in_p.filename();

		//std::cout << entry << '\n' << in_p << '\n' << out_p << '\n';

		// Read in point cloud into cloud_xyzrgbnormals
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgbnormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::io::loadPLYFile(in_p.string(), *cloud_xyzrgbnormals);

		// Compute reconstructed mesh
		pcl::PolygonMesh output;
		compute_mesh(cloud_xyzrgbnormals, output, default_depth, default_solver_divide, default_iso_divide, default_point_weight);

		// Save reconstructed mesh to ply
		pcl::io::savePLYFile(out_p.string(), output);
	}

	return (0);
}