#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/texture_mapping.h>
#include <volcap/io/io_cam.h>
#include <volcap/viz/viz.h>
#include <array>
#include <string>

// get the project directory from preprocessor set in the top-level CMakeLists.txt
const std::string PROJECT_DIR = _PROJECT_DIR;

// gets whatever comes before the first underscore (e.g. "2_d4153_30512.ply" => "2")
std::string get_frame_prefix(const char arr[]) {
	char* copy = strdup(arr);
	strtok(copy, "_");
	std::string str(copy);
	return str;
}

// builds length 8 filename with leading zeros (e.g. 4 => "00000004")
std::string build_ordered_filename(std::string idx) {
	std::ostringstream ss;
	ss << std::setw(8) << std::setfill('0') << idx;
	return ss.str();
}

void estimate_normals(
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
	pcl::PointCloud<pcl::PointNormal> &out_cloud,
	volcap::io::Camera &cam
) {

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	ne.setInputCloud(in_cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.03);  // Use all neighbors in a sphere of radius 3cm
	Eigen::Vector3f location = cam.getLocation();

	ne.setViewPoint(location(0), location(1), location(2));

	// Compute the features
	ne.compute(out_cloud);

	// concatenate the xyz info with the normals
	pcl::concatenateFields(out_cloud, *in_cloud, out_cloud);
}

// merges pointclouds p1-p4, crops them, writes result in `filtered_cloud`
void merge_and_crop(
	std::string p1,
	std::string p2,
	std::string p3,
	std::string p4,
	pcl::PointCloud<pcl::PointNormal> &filtered_cloud,
	std::vector<volcap::io::Camera*> cams
) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>());

	// load in point cloud for each cam
	pcl::io::loadPLYFile(p1, *cloud0);
	pcl::io::loadPLYFile(p2, *cloud1);
	pcl::io::loadPLYFile(p3, *cloud2);
	pcl::io::loadPLYFile(p4, *cloud3);

	// estimate normals
	pcl::PointCloud<pcl::PointNormal> cloud0_normals;
	pcl::PointCloud<pcl::PointNormal> cloud1_normals;
	pcl::PointCloud<pcl::PointNormal> cloud2_normals;
	pcl::PointCloud<pcl::PointNormal> cloud3_normals;
	estimate_normals(cloud0, cloud0_normals, *cams[0]);
	estimate_normals(cloud1, cloud1_normals, *cams[1]);
	estimate_normals(cloud2, cloud2_normals, *cams[2]);
	estimate_normals(cloud3, cloud3_normals, *cams[3]);


	// merge the 4 clouds
	pcl::PointCloud<pcl::PointNormal>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointNormal>());
	*merged_cloud += cloud0_normals;
	*merged_cloud += cloud1_normals;
	*merged_cloud += cloud2_normals;
	*merged_cloud += cloud3_normals;

	// crop inner cylinder -- x^2/r^2 + z^2/r^2 = 1
	pcl::ConditionAnd<pcl::PointNormal>::Ptr crop_condition(new pcl::ConditionAnd<pcl::PointNormal>());

	Eigen::Matrix3f A;
	float radius = 0.7;
	float radius_sqr = radius * radius;
	A << 1 / radius_sqr, 0, 0,		// 1 / 0.5**2 = 4
		0, 0, 0,
		0, 0, 1 / radius_sqr;
	Eigen::Vector3f v;  // [0, 0, 0]
	v << 0, 0, 0;
	float c = -1.0;

	pcl::TfQuadraticXYZComparison<pcl::PointNormal>::Ptr quad_comp(new pcl::TfQuadraticXYZComparison<pcl::PointNormal>(
		pcl::ComparisonOps::LT, A, v, c
		));

	crop_condition->addComparison(quad_comp);

	// crop floor (everything below y = -0.82)
	pcl::FieldComparison<pcl::PointNormal>::Ptr floor_plane_comp(new pcl::FieldComparison<pcl::PointNormal>(
		"y", pcl::ComparisonOps::GT, -0.8
		));

	crop_condition->addComparison(floor_plane_comp);


	// build the filter
	pcl::ConditionalRemoval<pcl::PointNormal> condrem;
	condrem.setCondition(crop_condition);
	condrem.setInputCloud(merged_cloud);

	// get filtered cloud
	condrem.filter(filtered_cloud);
}

int main(int argc, char** argv)
{
	std::string pc_dir = "D:/mhudnell/data/volumetric_recordings/4-23/pointcloud";
	std::string pc_out_dir = "D:/mhudnell/data/volumetric_recordings/4-23/pointcloud_merged/";
	std::string calib_dir = "D:/mhudnell/data/volumetric_recordings/4-23/calibration";

	std::string cam_cloud_paths[4];
	std::string cam_names[4];
	cam_names[0] = "d4150";
	cam_names[1] = "d4151";
	cam_names[2] = "d4152";
	cam_names[3] = "d4153";

	//pcl::texture_mapping::CameraVector cams;
	std::vector<volcap::io::Camera*> cams;
	volcap::io::loadCameraParams_VCL(calib_dir, cams);

	// get number of files in dir
	int merge_cnt_total = std::count_if(
		boost::filesystem::directory_iterator(pc_dir),
		boost::filesystem::directory_iterator(),
		static_cast<bool(*)(const boost::filesystem::path&)>(boost::filesystem::is_regular_file)) / 4;

	boost::filesystem::directory_iterator it{ pc_dir };
	std::string str_frame = "";  // the current frame idx we're merging. is not strictly increasing
	int merge_cnt = 1;  // the current idx of the output merged cloud. strictly increases by 1
	int idx_cam = 0;  // idx of current cam to look for. in [0, 3].
	while (it != boost::filesystem::directory_iterator{}) {
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path path = entry.path();

		// skip if entry is a directory
		if (!boost::filesystem::is_regular_file(path))
			continue;

		std::string filename = path.filename().string();
		if (idx_cam == 0) {  // starting new frame
			str_frame = get_frame_prefix(filename.c_str());
			printf("merging clouds %u/%u: %08u.ply\n", merge_cnt, merge_cnt_total, std::stoi(str_frame));
		}

		std::string desired_prefix = str_frame + "_" + cam_names[idx_cam];

		if (filename.rfind(desired_prefix, 0) == 0) {  // if filename starts with desired_prefix
			cam_cloud_paths[idx_cam] = path.string();
			idx_cam++;
		}

		if (idx_cam == 4) {
			pcl::PointCloud<pcl::PointNormal>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointNormal>);
			merge_and_crop(cam_cloud_paths[0], cam_cloud_paths[1], cam_cloud_paths[2], cam_cloud_paths[3], *merged_cloud, cams);

			// visualize for debug
			//volcap::viz::viz_pc<pcl::PointNormal>(merged_cloud, cams);

			std::string out_file = pc_out_dir + build_ordered_filename(str_frame) + ".ply";
			pcl::io::savePLYFile(out_file, *merged_cloud);

			idx_cam = 0;
			merge_cnt++;
		}
	}
	

}
