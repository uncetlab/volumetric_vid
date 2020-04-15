#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/conditional_removal.h>

// get the project directory from preprocessor set in the top-level CMakeLists.txt
const std::string PROJECT_DIR = _PROJECT_DIR;

int main(int argc, char** argv)
{
	
	std::string d4150_cloud = "D:/mhudnell/data/volumetric_recordings/pointcloud/0_d4150_30466.ply";
	std::string d4151_cloud = "D:/mhudnell/data/volumetric_recordings/pointcloud/0_d4151_29757.ply";
	std::string d4152_cloud = "D:/mhudnell/data/volumetric_recordings/pointcloud/0_d4152_28819.ply";
	std::string d4153_cloud = "D:/mhudnell/data/volumetric_recordings/pointcloud/0_d4153_30509.ply";

	pcl::PointCloud<pcl::PointXYZ> cloud0;
	pcl::PointCloud<pcl::PointXYZ> cloud1;
	pcl::PointCloud<pcl::PointXYZ> cloud2;
	pcl::PointCloud<pcl::PointXYZ> cloud3;

	// load in point cloud for each cam
	pcl::io::loadPLYFile(d4150_cloud, cloud0);
	pcl::io::loadPLYFile(d4151_cloud, cloud1);
	pcl::io::loadPLYFile(d4152_cloud, cloud2);
	pcl::io::loadPLYFile(d4153_cloud, cloud3);

	// merge the 4 clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	*merged_cloud += cloud0;
	*merged_cloud += cloud1;
	*merged_cloud += cloud2;
	*merged_cloud += cloud3;

	// crop inner cylinder -- x^2/r^2 + z^2/r^2 = 1
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr crop_condition(new pcl::ConditionAnd<pcl::PointXYZ>);

	Eigen::Matrix3f A;
	float radius = 0.7;
	float radius_sqr = radius * radius;
	A << 1/radius_sqr, 0, 0,		// 1 / 0.5**2 = 4
		0, 0, 0,
		0, 0, 1/radius_sqr;
	Eigen::Vector3f v;  // [0, 0, 0]
	float c = -1.0;

	pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::Ptr quad_comp(new pcl::TfQuadraticXYZComparison<pcl::PointXYZ>(
		pcl::ComparisonOps::LT, A, v, c
	));

	crop_condition->addComparison(quad_comp);

	// crop floor (everything below y = -0.82)
	pcl::FieldComparison<pcl::PointXYZ>::Ptr floor_plane_comp(new pcl::FieldComparison<pcl::PointXYZ>(
		"y", pcl::ComparisonOps::GT, -0.8
	));

	crop_condition->addComparison(floor_plane_comp);


	// build the filter
	pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
	condrem.setCondition(crop_condition);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	condrem.setInputCloud(merged_cloud);
	//condrem.setKeepOrganized(true);

	// get filtered cloud
	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
	condrem.filter(filtered_cloud);

	// write to .ply

	std::string file_name = "D:/mhudnell/data/volumetric_recordings/pointcloud/0_merge_crop.ply";
	pcl::io::savePLYFile(file_name, filtered_cloud);

}
