#include <pcl/surface/texture_mapping.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <stdio.h>
#include "texturing.h"

namespace pt = boost::property_tree;

// helper function to retrieve an element at a specific index within a ptree
template <typename T>
T element_at(pt::ptree const& pt, std::string name, size_t n) {
	return std::next(pt.get_child(name).find(""), n)->second.get_value<T>();
}

/** \brief Display a 3D representation showing the a cloud and a list of camera with their 6DOf poses */
void showCameras(pcl::texture_mapping::CameraVector cams, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

	// visualization object
	pcl::visualization::PCLVisualizer visu("cameras");

	// add a visual for each camera at the correct pose
	for (int i = 0; i < cams.size(); ++i)
	//for (int i = 0; i < 1; ++i)
	{
		// read current camera
		pcl::TextureMapping<pcl::PointXYZ>::Camera cam = cams[i];
		double focal = cam.focal_length;
		double focal_h = cam.focal_length_h;
		double focal_w = cam.focal_length_w;
		double height = cam.height;
		double width = cam.width;

		float scale = 50.0;
		//float scale = 1.0;
		// create a 5-point visual for each camera
		pcl::PointXYZ p1, p2, p3, p4, p5;
		p1.x = 0; p1.y = 0; p1.z = 0;
		double dist = 0.75;
		double angleX, angleY;
		double minX, minY, maxX, maxY;

		if (focal_h != -1 & focal_w != -1) {
			maxX = dist * tan(atan(width / (2.0*focal_w)));
			maxY = dist * tan(atan(height / (2.0*focal_h)));
			angleX = RAD2DEG(2.0 * atan(width / (2.0*focal_w)));
			angleY = RAD2DEG(2.0 * atan(height / (2.0*focal_h)));
		}
		else {
			maxX = dist * tan(atan(width / (2.0*focal)));
			maxY = dist * tan(atan(height / (2.0*focal)));
			angleX = RAD2DEG(2.0 * atan(width / (2.0*focal)));
			angleY = RAD2DEG(2.0 * atan(height / (2.0*focal)));
		}
		minX = -maxX;
		minY = -maxY;
		
		//p2.x = minX; p2.y = minY; p2.z = dist;
		//p3.x = maxX; p3.y = minY; p3.z = dist;
		//p4.x = maxX; p4.y = maxY; p4.z = dist;
		//p5.x = minX; p5.y = maxY; p5.z = dist;
		p2.x = minX * scale; p2.y = minY * scale; p2.z = dist * scale;
		p3.x = maxX * scale; p3.y = minY * scale; p3.z = dist * scale;
		p4.x = maxX * scale; p4.y = maxY * scale; p4.z = dist * scale;
		p5.x = minX * scale; p5.y = maxY * scale; p5.z = dist * scale;
		p1 = pcl::transformPoint(p1, cam.pose);
		p2 = pcl::transformPoint(p2, cam.pose);
		p3 = pcl::transformPoint(p3, cam.pose);
		p4 = pcl::transformPoint(p4, cam.pose);
		p5 = pcl::transformPoint(p5, cam.pose);
		std::stringstream ss;
		ss << "Cam #" << i + 1;
		visu.addText3D(ss.str(), p1, 0.1, 1.0, 1.0, 1.0, ss.str());

		ss.str("");
		ss << "camera_" << i << "line1";
		visu.addLine(p1, p2, ss.str());
		ss.str("");
		ss << "camera_" << i << "line2";
		visu.addLine(p1, p3, ss.str());
		ss.str("");
		ss << "camera_" << i << "line3";
		visu.addLine(p1, p4, ss.str());
		ss.str("");
		ss << "camera_" << i << "line4";
		visu.addLine(p1, p5, ss.str());
		ss.str("");
		ss << "camera_" << i << "line5";
		visu.addLine(p2, p5, ss.str());
		ss.str("");
		ss << "camera_" << i << "line6";
		visu.addLine(p5, p4, ss.str());
		ss.str("");
		ss << "camera_" << i << "line7";
		visu.addLine(p4, p3, ss.str());
		ss.str("");
		ss << "camera_" << i << "line8";
		visu.addLine(p3, p2, ss.str());
	}

	// add a coordinate system
	visu.addCoordinateSystem(100.0);

	// add the mesh's cloud (colored on Z axis)
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(cloud, "z");
	visu.addPointCloud(cloud, color_handler, "cloud");

	// reset camera
	visu.resetCamera();

	// wait for user input
	visu.spin();
}

void loadCameraParams(std::string calibration_file, pcl::texture_mapping::CameraVector &cam_list, std::vector<std::string>* cams_to_use = nullptr) {
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
		Eigen::Vector3f location = -orientation*t;
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

/* \brief Manually sets a camera's params (for testing)
 *
 * \param[out] cam Camera that gets manually set.
 */
void hardcodeLoadCameraParam(pcl::TextureMapping<pcl::PointXYZ>::Camera &cam) {
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

// custom segmentation + texturing demo
void custom_seg_demo() {
	//==> load TextureMesh with UVAtlas' uv-mapping
	pcl::TextureMesh tmesh;
	pcl::io::loadOBJFile("C:/Users/maxhu/Desktop/uvatlas_example/texture_mapping_tests/test_panoptic.obj", tmesh);

	//==> load cameras
	pcl::texture_mapping::CameraVector my_cams;
	loadCameraParams("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/calibration_171026_pose3.json", my_cams);

	//==> segment using custom func
	std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> tex_coords;
	std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> img_coords;
	segmentUVMeshByCamera(tmesh, my_cams, tex_coords, img_coords);

	//==> prepare image files
	std::vector<std::string> img_files;
	for (int cam_idx = 0; cam_idx < my_cams.size(); cam_idx++) {
		img_files.push_back(my_cams[cam_idx].texture_file);
	}

	//==> generate texture map using UVAtlas' uv-map, greedy custom segmentation
	//std::string texture_file_name = "C:/Users/maxhu/Desktop/uvatlas_example/texture_mapping_tests/test_panoptic_texture_all.bmp";
	std::string texture_file_name = "C:/Users/maxhu/Desktop/uvatlas_example/texture_mapping_tests/test_panoptic_texture_04.bmp";
	generateUVTextureFromImages(texture_file_name, tex_coords, img_coords, img_files);
}

// PCL's textureMeshwithMultipleCameras() demo
// textureMeshwithMultipleCameras does camera segmentation and the uv mapping for each camera
void pcl_texture_demo() {
	//====> prepare a texturemesh for input
	// load a PolygonMesh (no uv coordinates yet, textureMeshwithMultipleCameras() will generate them per camera)
	pcl::PolygonMesh pmesh;
	std::string fname = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_decimated_0.900000/ptcloud_hd00000380_normals_cleaned.ply";
	PCL_INFO("\nLoading mesh from file %s...\n", fname.c_str());
	pcl::io::loadPLYFile(fname, pmesh);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pmesh.cloud, *cloud);

	// Create the texturemesh object that will contain our UV-mapped mesh
	pcl::TextureMesh tmesh;
	tmesh.cloud = pmesh.cloud;
	std::vector< pcl::Vertices> polygon_1;

	// push faces into the texturemesh object (copies pmesh.polygons into polygon_1)
	polygon_1.resize(pmesh.polygons.size());
	for (size_t i = 0; i < pmesh.polygons.size(); ++i)
	{
		polygon_1[i] = pmesh.polygons[i];
	}
	tmesh.tex_polygons.push_back(polygon_1);
	PCL_INFO("\tInput mesh contains %d faces and %d vertices\n", tmesh.tex_polygons[0].size(), cloud->points.size());
	PCL_INFO("...Done.\n");


	//====> load cameras from file
	pcl::texture_mapping::CameraVector my_cams;

	//// load all cams
	//loadCameraParams("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/calibration_171026_pose3.json", my_cams);

	// or specify just a single cam
	std::vector<std::string> cams_to_use;  // list of camera names to get loaded
	cams_to_use.push_back("50_02");
	loadCameraParams("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/calibration_171026_pose3.json", my_cams, &cams_to_use);

	//// or just manually set one camera
	//pcl::TextureMapping<pcl::PointXYZ>::Camera &cam;
	//hardcodeLoadCameraParam(cam);
	//my_cams.push_back(cam);

	////====> Display cameras to user
	//PCL_INFO("\nDisplaying cameras. Press \'q\' to continue texture mapping\n");
	//showCameras(my_cams, cloud);

	//====> Create materials for each camera (and one extra for occluded faces)
	tmesh.tex_materials.resize(my_cams.size() + 1);
	for (int i = 0; i <= my_cams.size(); ++i)
	{
		pcl::TexMaterial mesh_material;
		mesh_material.tex_Ka.r = 0.2f;
		mesh_material.tex_Ka.g = 0.2f;
		mesh_material.tex_Ka.b = 0.2f;

		mesh_material.tex_Kd.r = 0.8f;
		mesh_material.tex_Kd.g = 0.8f;
		mesh_material.tex_Kd.b = 0.8f;

		mesh_material.tex_Ks.r = 1.0f;
		mesh_material.tex_Ks.g = 1.0f;
		mesh_material.tex_Ks.b = 1.0f;

		mesh_material.tex_d = 1.0f;
		mesh_material.tex_Ns = 75.0f;
		mesh_material.tex_illum = 2;

		std::stringstream tex_name;
		tex_name << "material_" << i;
		tex_name >> mesh_material.tex_name;

		if (i < my_cams.size())
			mesh_material.tex_file = my_cams[i].texture_file;
		else
			mesh_material.tex_file = "C:/Users/maxhu/Desktop/uvatlas_example/occluded.jpg";

		tmesh.tex_materials[i] = mesh_material;
	}

	//====> Segment mesh and create uv coords for each camera
	PCL_INFO("\nSorting faces by cameras...\n");
	pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
	tm.textureMeshwithMultipleCameras(tmesh, my_cams);
	PCL_INFO("Sorting faces by cameras done.\n");

	//for (int i = 0; i <= my_cams.size(); ++i)
	//{
	//	PCL_INFO("\tSub mesh %d contains %d faces and %d UV coordinates.\n", i, tmesh.tex_polygons[i].size(), tmesh.tex_coordinates[i].size());
	//}

	//====> Compute normals for the mesh
	PCL_INFO("\nEstimating normals...\n");
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	//====> Concatenate XYZ and normal fields, put back into TextureMesh
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	pcl::toPCLPointCloud2(*cloud_with_normals, tmesh.cloud);
	PCL_INFO("...Done.\n");


	//====> save TextureMesh to obj
	std::string file_name = "pcl_texturing_50_02.obj";
	std::string save_message = "\nSaving mesh to " + file_name + "\n";
	PCL_INFO(save_message.c_str());
	pcl::io::saveOBJFile("C:/Users/maxhu/Desktop/uvatlas_example/texture_mapping_tests/"+ file_name, tmesh);  // MUST be declared with forward slashes to work correctly
}

//void pcl_segmentation_with_custom_texture_demo() {
//	//==> load TextureMesh with UVAtlas' uv-mapping
//	pcl::TextureMesh tmesh;
//	pcl::io::loadOBJFile("C:/Users/maxhu/Desktop/uvatlas_example/test_panoptic.obj", tmesh);
//
//	//==> segment by camera visibility (USING PCL's FUNCS)
//	pcl::TextureMapping<pcl::PointXYZ> tm;
//
//	//pcl::TextureMesh sorted_tmesh;
//	//pcl::TextureMapping<pcl::PointXYZ>::PointCloud visible_pts;
//	//// this actually deletes faces from tmesh.tex_polygons
//	//// this also segments faces in a greedy manner (no face is seen by more than 1 cam)
//	//// this copies tex_coordinates / tex_materials directly from tmesh into sorted_tmesh (does NOT resort tex_coordinates)
//	//tm.sortFacesByCamera(tmesh, sorted_tmesh, my_cams, 1.0, visible_pts); // DOES NOT resort tex_coordinates
//
//	tm.textureMeshwithMultipleCameras(tmesh, my_cams);						// OVERWRITES tex_coordinates with 3d faces projected on 2d camera planes!
//
//	//==> project points to 2d planes to get img coords
//	std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> img_coords;
//	//std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> img_coords;
//
//	// change to PointCloud for easier access
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ> cloud;
//	pcl::fromPCLPointCloud2(tmesh.cloud, cloud);
//
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
//
//
//	// loop thru submeshes (aka cameras)
//	int invis_face_count = 0;
//	for (int submesh_idx=0; submesh_idx < tmesh.tex_polygons.size()-1; submesh_idx++)
//	{
//		std::vector<pcl::Vertices> &submesh = tmesh.tex_polygons[submesh_idx];
//		std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> submesh_img_coords;
//
//		// transform original cloud in camera coordinates
//		Eigen::Affine3f cam_trans = my_cams[submesh_idx].pose;
//		pcl::transformPointCloud(cloud, transformed_cloud, cam_trans.inverse());
//
//		for (int face_idx = 0; face_idx < submesh.size(); face_idx++) {
//			pcl::Vertices &face = submesh[face_idx];
//
//			int point_idx_0 = face.vertices[0];
//			int point_idx_1 = face.vertices[1];
//			int point_idx_2 = face.vertices[2];
//
//			Eigen::Vector2f img_coord_0;
//			Eigen::Vector2f img_coord_1;
//			Eigen::Vector2f img_coord_2;
//
//			// determine 2d camera coordinates of a 3d face
//			//tm.getPointUVCoordinates(cloud[point_idx_0], my_cams[submesh_idx], img_coord_0);
//			//tm.getPointUVCoordinates(cloud[point_idx_1], my_cams[submesh_idx], img_coord_1);
//			//tm.getPointUVCoordinates(cloud[point_idx_2], my_cams[submesh_idx], img_coord_2);
//
//			tm.getPointUVCoordinates(transformed_cloud[point_idx_0], my_cams[submesh_idx], img_coord_0);
//			tm.getPointUVCoordinates(transformed_cloud[point_idx_1], my_cams[submesh_idx], img_coord_1);
//			tm.getPointUVCoordinates(transformed_cloud[point_idx_2], my_cams[submesh_idx], img_coord_2);
//
//			Eigen::Vector2f invis_coord(-1.0, -1.0);
//			if (img_coord_0 == invis_coord || img_coord_1 == invis_coord || img_coord_2 == invis_coord) {
//				printf("ERROR cam %i: faces should be visible\n", submesh_idx);
//				invis_face_count++;
//			} else {
//				submesh_img_coords.push_back(img_coord_0);
//				submesh_img_coords.push_back(img_coord_1);
//				submesh_img_coords.push_back(img_coord_2);
//			}
//		}
//
//		img_coords.push_back(submesh_img_coords);
//
//	}
//	printf("invis_face_count: %i\n", invis_face_count);
//
//	//==> prepare image files
//	std::vector<std::string> img_files;
//	for (int cam_idx = 0; cam_idx < my_cams.size(); cam_idx++) {
//		img_files.push_back(my_cams[cam_idx].texture_file);
//	}
//
//	//==> generate texture map
//	std::string texture_file_name = "uvatlas_texture_map.bmp";
//	//generateUVTextureFromImages(texture_file_name, sorted_tmesh.tex_coordinates, img_coords, img_files);
//	generateUVTextureFromImages(texture_file_name, tmesh.tex_coordinates, img_coords, img_files);
//}

// copying image textures onto UVAtlas' uv-map, using greedy PCL segmentation from textureMeshwithMultipleCameras
int main(int argc, char** argv) {

	custom_seg_demo();

	//pcl_texture_demo();

	//pcl_segmentation_with_custom_texture_demo();
	
}