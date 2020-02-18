#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/io/ply_io.h>
#include "SPSR.h"
//#include "../SPSR/SPSR.h"

//const int MIN_NEIGHBORS = 0; // required number of nearest neighbors
//const float MIN_RADIUS = 5; // minimum radius of sphere

int main(int argc, char** argv)
{
	// loop through clean point clouds
	boost::filesystem::path input_dir("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/normals_cleaned");
	boost::filesystem::directory_iterator it{ input_dir };
	while (it != boost::filesystem::directory_iterator{}) {

		// Get input paths
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path in_p = entry.path();

		// Read in point cloud into cloud_xyzrgbnormals
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgbnormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::io::loadPLYFile(in_p.string(), *cloud_xyzrgbnormals);

		// set mls params
		pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
		pcl::MovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> mls;
		mls.setSearchMethod(tree);
		mls.setInputCloud(cloud_xyzrgbnormals);
		mls.setSearchRadius(3);
		//mls.setSearchRadius(MIN_RADIUS);
		//mls.setComputeNormals(true);  // TODO: compute new normals and flip them to be correct instead of using unsmoothed normals

		// compute mls
		//pcl::PointCloud<pcl::PointNormal>::Ptr xyz_cloud_smoothed(new pcl::PointCloud<pcl::PointNormal>());
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr xyz_cloud_smoothed(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
		mls.process(*xyz_cloud_smoothed);

		//// Write output to ply
		////std::string fname_out = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/ptcloud_hd00000380_orig_normals_"+ std::to_string(i) +".ply";
		//boost::filesystem::path out_p = in_p.parent_path().parent_path() / "cloud_mls" / in_p.filename();
		//pcl::io::savePLYFileASCII(out_p.string(), *xyz_cloud_smoothed);

		// compute spsr
		pcl::PolygonMesh::Ptr psr_mesh(boost::make_shared<pcl::PolygonMesh>());
		compute_mesh(xyz_cloud_smoothed, *psr_mesh);
		//pcl::PointCloud<pcl::PointXYZRGBNormal> points;
		//std::vector<pcl::Vertices> polygons;
		//compute_mesh(xyz_cloud_smoothed, points, polygons);

		// decimate mesh
		pcl::PolygonMesh decimated_mesh;
		float percent_faces_removed = 0.9;
		decimate_mesh(psr_mesh, percent_faces_removed, decimated_mesh);

		// Write output to ply
		std::string decimated_folder_str = "decimated_";
		decimated_folder_str += std::to_string(percent_faces_removed);

		//boost::filesystem::path out_p = in_p.parent_path().parent_path() / "cloud_mls" / "full" / in_p.filename();
		boost::filesystem::path out_p = in_p.parent_path().parent_path() / "cloud_mls" / decimated_folder_str / in_p.filename();
		boost::filesystem::create_directory(out_p.parent_path());

		//pcl::io::savePLYFile(out_p.string(), *psr_mesh);
		pcl::io::savePLYFile(out_p.string(), decimated_mesh);
	}

	//// loop thru different levels of smoothing
	//for (int i = 1; i < 6; i++) {
	//	mls.setSearchRadius(i);
	//	mls.process(*xyz_cloud_smoothed);

	//	// Write output to ply
	//	std::string fname_out = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/ptcloud_hd00000380_orig_normals_"+ std::to_string(i) +".ply";
	//	pcl::io::savePLYFileASCII(fname_out, *xyz_cloud_smoothed);
	//}

}
