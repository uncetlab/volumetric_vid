#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/ply_io.h>

int main(int argc, char** argv)
{

	// Read in point cloud into cloud_xyzrgbnormals
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgbnormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	const std::string fname = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/ptcloud_hd00000380_normals_cleaned.ply";
	pcl::io::loadPLYFile(fname, *cloud_xyzrgbnormals);

	// output mesh
	const std::string fname_out = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/ptcloud_hd00000380_pcl_mesh.ply";
	pcl::PolygonMesh output;

	int default_depth = 8;
	int default_solver_divide = 8;
	int default_iso_divide = 8;
	float default_point_weight = 4.0f;

	pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
	poisson.setDepth(default_depth);
	poisson.setSolverDivide(default_solver_divide);
	poisson.setIsoDivide(default_iso_divide);
	poisson.setPointWeight(default_point_weight);
	poisson.setInputCloud(cloud_xyzrgbnormals);
	poisson.reconstruct(output);

	// save reconstructed mesh to ply
	pcl::io::savePLYFile(fname_out, output);

	return (0);
}