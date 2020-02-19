#include <volcap/io/io_cam.h>
#include <pcl/surface/texture_mapping.h>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <vector>
#include <string>

namespace pt = boost::property_tree;

// helper function to retrieve an element at a specific index within a ptree
template <typename T>
T element_at(pt::ptree const& pt, std::string name, size_t n) {
	return std::next(pt.get_child(name).find(""), n)->second.get_value<T>();
}

void volcap::io::loadCameraParams(std::string calibration_file, pcl::texture_mapping::CameraVector &cam_list, std::vector<std::string>* cams_to_use) {
	pt::ptree root;
	pt::read_json(calibration_file, root);

	for (pt::ptree::value_type &cam : root.get_child("cameras"))
	{
		pt::ptree cam_tree = cam.second;
		std::string type = cam_tree.get<std::string>("type");
		std::string name = cam_tree.get<std::string>("name");


		// skip unwanted devices
		if (type != "kinect-color" ||
			(cams_to_use != nullptr && std::find(cams_to_use->begin(), cams_to_use->end(), name) == cams_to_use->end())
			) {
			continue;
		}

		//printf("\nname: %s\n", name.c_str());

		// load rotation matrix R from json
		Eigen::Matrix3f R;
		int x = 0;
		for (pt::ptree::value_type &row : cam_tree.get_child("R")) {
			int y = 0;
			for (pt::ptree::value_type &cell : row.second) {
				//printf("%f ", cell.second.get_value<float>());
				R(x, y) = cell.second.get_value<float>();
				y++;
			}
			//printf("\n");
			x++;
		}

		// load translation vector t from json
		Eigen::Vector3f t;
		int i = 0;
		for (pt::ptree::value_type &row : cam_tree.get_child("t")) {
			for (pt::ptree::value_type &cell : row.second) {
				//printf("%f ", cell.second.get_value<float>());
				t(i) = cell.second.get_value<float>();
			}
			i++;
		}
		//printf("\n");

		// load intrinsic matrix K from json
		Eigen::Matrix3f K;
		x = 0;
		for (pt::ptree::value_type &row : cam_tree.get_child("K")) {
			int y = 0;
			for (pt::ptree::value_type &cell : row.second) {
				//printf("%f ", cell.second.get_value<float>());
				K(x, y) = cell.second.get_value<float>();
				y++;
			}
			//printf("\n");
			x++;
		}

		Eigen::Matrix3f orientation = R.transpose();
		Eigen::Vector3f location = -orientation * t;
		//std::cout << "R^T =\n" << orientation << std::endl;
		//std::cout << "-R^T*t =\n" << location << std::endl;

		// set Camera variables
		pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
		// location
		cam.pose(0, 3) = location(0); //TX
		cam.pose(1, 3) = location(1); //TY
		cam.pose(2, 3) = location(2); //TZ

		// orientation
		cam.pose(0, 0) = orientation(0, 0);
		cam.pose(0, 1) = orientation(0, 1);
		cam.pose(0, 2) = orientation(0, 2);
		cam.pose(1, 0) = orientation(1, 0);
		cam.pose(1, 1) = orientation(1, 1);
		cam.pose(1, 2) = orientation(1, 2);
		cam.pose(2, 0) = orientation(2, 0);
		cam.pose(2, 1) = orientation(2, 1);
		cam.pose(2, 2) = orientation(2, 2);

		// scale
		cam.pose(3, 0) = 0.0;
		cam.pose(3, 1) = 0.0;
		cam.pose(3, 2) = 0.0;
		cam.pose(3, 3) = 1.0;

		// load cam resolution from json
		cam.height = element_at<double>(cam_tree, "resolution", 1);
		cam.width = element_at<double>(cam_tree, "resolution", 0);
		cam.focal_length_h = K(1, 1);
		cam.focal_length_w = K(0, 0);
		cam.center_h = K(1, 2);
		cam.center_w = K(0, 2);

		// texture
		// determine texture path -- for all images? just 380 for now
		boost::filesystem::path calibration_path = calibration_file;
		boost::filesystem::path texture_path = calibration_path.parent_path() / "kinectImgs" / name / (name + "_00000380.jpg");

		cam.texture_file = texture_path.string();
		//cam.texture_file = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinectImgs/50_04/50_04_00000380.jpg";

		cam_list.push_back(cam);
	}
}

void volcap::io::hardcodeLoadCameraParam(pcl::TextureMapping<pcl::PointXYZ>::Camera &cam) {
	cam.texture_file = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinectImgs/50_04/50_04_00000380.jpg";

	// location
	cam.pose(0, 3) = 224.20499997; //TX
	cam.pose(1, 3) = -104.32799999; //TY
	cam.pose(2, 3) = 141.55799999; //TZ

	// orientation
	cam.pose(0, 0) = -0.4791;
	cam.pose(0, 1) = -0.0565;
	cam.pose(0, 2) = -0.8759;
	cam.pose(1, 0) = -0.0122;
	cam.pose(1, 1) = 0.9983;
	cam.pose(1, 2) = -0.0578;
	cam.pose(2, 0) = 0.8777;
	cam.pose(2, 1) = -0.0170;
	cam.pose(2, 2) = -0.4790;

	// scale
	cam.pose(3, 0) = 0.0;
	cam.pose(3, 1) = 0.0;
	cam.pose(3, 2) = 0.0;
	cam.pose(3, 3) = 1.0;

	// focal length, camera resolution, camera center (aka principle point offset)
	cam.focal_length = 1052.37;
	//cam.focal_length_h = 1052.37;
	//cam.focal_length_w = 1052.37;
	cam.height = 1080;
	cam.width = 1920;
	cam.center_h = 535.367;
	cam.center_w = 953.062;
}
