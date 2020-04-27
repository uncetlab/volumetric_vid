#include <volcap/io/io_cam.h>
#include <pcl/surface/texture_mapping.h>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <vector>
#include <string>
#include <stdexcept>

#include <iostream>
#include <fstream>
#include <sstream>

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

void volcap::io::loadCameraParams_VCL(std::string calibration_dir, std::vector<volcap::io::Camera*> &cam_list) {

	// could do something like this to fix alphabetical file ordering problem
	//// initalize a cam for each d415
	//for (int i = 0; i < 4; i++) {
	//	//pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
	//	volcap::io::Camera cam;
	//	cam_list.push_back(cam);
	//}

	boost::filesystem::path calibration_dir_path(calibration_dir);
	boost::filesystem::directory_iterator it{ calibration_dir_path };
	while (it != boost::filesystem::directory_iterator{}) {
		// Get input / output paths
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path path = entry.path();
		std::string file_path = path.string();
		std::string file_stem = path.stem().string();

		if (boost::filesystem::extension(path) == ".extrinsics") {
			std::ifstream f(file_path);
			std::string line;

			// load rotation matrix R from .txt
			Eigen::Matrix3f orientation;
			for (int i = 0; i < 3; i++) {  // loop thru lines
				std::getline(f, line);
				std::stringstream lineStream(line);
				for (int j = 0; j < 3; j++) {  // loop thru vals
					std::string val;
					std::getline(lineStream, val, ' ');
					orientation(i, j) = std::stof(val);  // removes any leftover '\n' at the end
				}
			}

			// load translation vector t from .txt
			Eigen::Vector3f location;
			for (int i = 0; i < 3; i++) {  // loop thru final row
				std::string val;
				std::getline(f, val, ' ');
				location(i) = std::stof(val) / 1000.0;  // convert millimeters to meters
			}

			// change z-up to y-up
			orientation.row(2).swap(orientation.row(1));
			orientation.row(2) *= -1;
			location.row(2).swap(location.row(1));
			location.row(2) *= -1;

			

			volcap::io::Camera *cam = new volcap::io::Camera(file_stem, orientation, location);
			cam_list.push_back(cam);
		}
		else if (path.filename().string() == "device_repository.json") {
			// let's assume all the .extrinsics files have already been viewed, 
			// and entries for each cam exist in cam_list

			// load intrinsic matrix K for call cams
			pt::ptree root;
			pt::read_json(file_path, root);

			int cam_idx = 0;
			for (pt::ptree::value_type& array_element : root) {
				pt::ptree& obj_i = array_element.second;

				Eigen::Matrix<float, 3, 3, Eigen::RowMajor> K;

				pt::ptree& color_intrinsics = obj_i.get_child("Color Intrinsics");
				for (pt::ptree::value_type& intrinsic_element : color_intrinsics) {
					pt::ptree& obj_j = intrinsic_element.second;
					try {
						pt::ptree& desired_res = obj_j.get_child("1280x720");
						int i = 0;
						for (pt::ptree::value_type& val : desired_res) {
							K(i++) = val.second.get_value<float>();
						}
					}
					catch (...) {

					}
				}

				// save K to cam_list entry
				std::string device = obj_i.get_child("Device").get_value<std::string>();
				if (cam_list.size() > cam_idx && cam_list[cam_idx]->name == device) {
					cam_list[cam_idx]->K = K;
					
					// set height and width
					cam_list[cam_idx]->width = 1280;
					cam_list[cam_idx]->height = 720;

					cam_idx++;
				}
				else {
					throw std::logic_error("Logic error: all `*.extrinsics` filenames must be alphabetically less than `device_repository.json`, so they are read first.");
				}
			}
		}
		else {
			printf("skipping file with extension: %s\n", boost::filesystem::extension(path).c_str());
			continue;
		}
	}

	// print loaded info
	for (int i = 0; i < cam_list.size(); i++) {
		volcap::io::Camera &cam = *cam_list[i];
		std::cout << "______________" << '\n';
		std::cout << cam.name << '\n';

		std::cout << "Orientation:" << '\n';
		std::cout << cam.orientation << '\n';
		std::cout << "Location:" << '\n';
		std::cout << cam.location << '\n';
		std::cout << "K:" << '\n';
		std::cout << cam.K << '\n';
		std::cout << "width: " << cam.width << '\n';
		std::cout << "height: " << cam.height << '\n';
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

pcl::texture_mapping::Camera volcap::io::Camera::toPCLCamera() {
	pcl::texture_mapping::Camera cam;

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

	// intrinsics
	cam.height = height;
	cam.width = width;
	cam.focal_length_h = K(1, 1);
	cam.focal_length_w = K(0, 0);
	cam.center_h = K(1, 2);
	cam.center_w = K(0, 2);

	return cam;
}

Eigen::Affine3f volcap::io::Camera::getPose() {
	Eigen::Affine3f pose;

	pose(0, 3) = location(0); //TX
	pose(1, 3) = location(1); //TY
	pose(2, 3) = location(2); //TZ

	pose(0, 0) = orientation(0, 0);
	pose(0, 1) = orientation(0, 1);
	pose(0, 2) = orientation(0, 2);
	pose(1, 0) = orientation(1, 0);
	pose(1, 1) = orientation(1, 1);
	pose(1, 2) = orientation(1, 2);
	pose(2, 0) = orientation(2, 0);
	pose(2, 1) = orientation(2, 1);
	pose(2, 2) = orientation(2, 2);

	pose(3, 0) = 0.0;
	pose(3, 1) = 0.0;
	pose(3, 2) = 0.0;
	pose(3, 3) = 1.0;

	return pose;
}
