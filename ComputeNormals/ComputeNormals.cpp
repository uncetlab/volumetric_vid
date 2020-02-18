#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <stdio.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>

void compute_normals() {
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);

	// Read in point cloud into cloud_xyzrgb
	const std::string fname = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/ptcloud_hd00000380_clipped_float.ply";
	pcl::io::loadPLYFile(fname, *cloud_xyzrgb);

	// convert PointXYZRGB to PointXYZ
	pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud_xyz);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal >::Ptr cloud_xyzrgbnormals(new pcl::PointCloud<pcl::PointXYZRGBNormal >);

	// Use all neighbors in a sphere of radius 3cm
	//ne.setRadiusSearch(0.03);
	ne.setKSearch(5);  // mh - 11.4.19

	// Compute the features
	ne.compute(*cloud_normals);

	// concatenate xyz cloud with new normals
	pcl::concatenateFields(*cloud_xyzrgb, *cloud_normals, *cloud_xyzrgbnormals);

	// Write output to ply
	const std::string fname_out = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/ptcloud_hd00000380_normals.ply";
	pcl::io::savePLYFileASCII(fname_out, *cloud_xyzrgbnormals);
}

void test_load() {
	//const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Read in point cloud  into cloud_xyzrgb
	const std::string fname = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/ptcloud_hd00000380_clipped_float.ply";
	pcl::io::loadPLYFile(fname, *cloud);

	// DEBUG: Write output to ply
	const std::string fname_out = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/ptcloud_hd00000380_xyzrgb.ply";
	pcl::io::savePLYFileASCII(fname_out, *cloud);
}

int main(int argc, char** argv)
{
	compute_normals();
	//test_load();

	return (0);
}
