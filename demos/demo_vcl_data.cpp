#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/conditional_removal.h>
#include <array>
#include <string>

// get the project directory from preprocessor set in the top-level CMakeLists.txt
const std::string PROJECT_DIR = _PROJECT_DIR;

// gets whatever comes before the first underscore (e.g. "2_d4153_30512.ply" => "2")
std::string get_frame_prefix(const char arr[]) {
	char* copy = strdup(arr);
	strtok(copy, "_");
	if (copy != NULL)
		printf("%s\n", copy);
	std::string str(copy);
	return str;
}

// merges pointclouds p1-p4, crops them, writes result in `filtered_cloud`
void merge_and_crop(
	std::string p1,
	std::string p2,
	std::string p3,
	std::string p4,
	pcl::PointCloud<pcl::PointXYZ> &filtered_cloud
) {

	pcl::PointCloud<pcl::PointXYZ> cloud0;
	pcl::PointCloud<pcl::PointXYZ> cloud1;
	pcl::PointCloud<pcl::PointXYZ> cloud2;
	pcl::PointCloud<pcl::PointXYZ> cloud3;

	// load in point cloud for each cam
	pcl::io::loadPLYFile(p1, cloud0);
	pcl::io::loadPLYFile(p2, cloud1);
	pcl::io::loadPLYFile(p3, cloud2);
	pcl::io::loadPLYFile(p4, cloud3);

	// merge the 4 clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	*merged_cloud += cloud0;
	*merged_cloud += cloud1;
	*merged_cloud += cloud2;
	*merged_cloud += cloud3;

	// crop inner cylinder -- x^2/r^2 + z^2/r^2 = 1
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr crop_condition(new pcl::ConditionAnd<pcl::PointXYZ>());

	Eigen::Matrix3f A;
	float radius = 0.7;
	float radius_sqr = radius * radius;
	A << 1 / radius_sqr, 0, 0,		// 1 / 0.5**2 = 4
		0, 0, 0,
		0, 0, 1 / radius_sqr;
	Eigen::Vector3f v;  // [0, 0, 0]
	v << 0, 0, 0;
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
	condrem.setInputCloud(merged_cloud);

	// get filtered cloud
	condrem.filter(filtered_cloud);
}

int main(int argc, char** argv)
{
	std::string pc_dir = "D:/mhudnell/data/volumetric_recordings/pointcloud";
	std::string pc_out_dir = "D:/mhudnell/data/volumetric_recordings/pointcloud/merged/";

	std::string cam_cloud_paths[4];
	std::string cam_names[4];
	cam_names[0] = "d4150";
	cam_names[1] = "d4151";
	cam_names[2] = "d4152";
	cam_names[3] = "d4153";

	boost::filesystem::directory_iterator it{ pc_dir };
	//int idx_frame = 0;
	std::string str_frame = "";
	int idx_cam = 0;
	while (it != boost::filesystem::directory_iterator{}) {
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path path = entry.path();

		// skip if entry is a directory
		if (!boost::filesystem::is_regular_file(path))
			continue;

		std::string filename = path.filename().string();
		if (idx_cam == 0) {  // starting new frame
			str_frame = get_frame_prefix(filename.c_str());
			//str_frame = get_frame_prefix(path.filename().c_str());
		}

		//std::string desired_prefix = std::to_string(idx_frame) + "_" + cam_names[idx_cam];
		std::string desired_prefix = str_frame + "_" + cam_names[idx_cam];

		if (filename.rfind(desired_prefix, 0) == 0) {  // if filename starts with desired_prefix
			cam_cloud_paths[idx_cam] = path.string();
			idx_cam++;
		}

		if (idx_cam == 4) {
			pcl::PointCloud<pcl::PointXYZ> merged_cloud;
			merge_and_crop(cam_cloud_paths[0], cam_cloud_paths[1], cam_cloud_paths[2], cam_cloud_paths[3], merged_cloud);

			// save to file
			//std::string out_file = pc_out_dir + std::to_string(idx_frame) + ".ply";
			std::string out_file = pc_out_dir + str_frame + ".ply";
			pcl::io::savePLYFile(out_file, merged_cloud);

			idx_cam = 0;
			//idx_frame++;
		}
	}
	

}
