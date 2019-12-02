#include <pcl/surface/mls.h>
#include <pcl/io/ply_io.h>

//const int MIN_NEIGHBORS = 0; // required number of nearest neighbors
//const float MIN_RADIUS = 5; // minimum radius of sphere

int main(int argc, char** argv)
{
	// load cleaned cloud
	std::string file_path = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/normals_cleaned/ptcloud_hd00000380_normals_cleaned.ply";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgbnormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::io::loadPLYFile(file_path, *cloud_xyzrgbnormals);

	// set mls params
	pcl::MovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointNormal> mls;
	mls.setInputCloud(cloud_xyzrgbnormals);
	//mls.setSearchRadius(MIN_RADIUS);

	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
	mls.setSearchMethod(tree);
	//mls.setComputeNormals(true);  // TODO: compute new normals and flip them to be correct instead of using unsmoothed normals

	// output cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr xyz_cloud_smoothed(new pcl::PointCloud<pcl::PointNormal>());

	// compute mls

	// loop thru different levels of smoothing
	for (int i = 1; i < 6; i++) {
		mls.setSearchRadius(i);
		mls.process(*xyz_cloud_smoothed);

		// Write output to ply
		std::string fname_out = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/ptcloud_hd00000380_orig_normals_"+ std::to_string(i) +".ply";
		pcl::io::savePLYFileASCII(fname_out, *xyz_cloud_smoothed);
	}

}
