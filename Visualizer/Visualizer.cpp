#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <boost/filesystem.hpp>
#include <vector>
#include <stdio.h>
//#include <type_traits>  // for checking template type

/* loads all PolygonMeshes (saved as .ply files) from a dir (assumes every file is a .ply file)
 *
 */
void load_meshes_from_dir(const std::string dir_name, std::vector<pcl::PolygonMeshPtr> &meshes, std::vector<std::string> &mesh_ids) {
	boost::filesystem::path input_dir(dir_name);

	boost::filesystem::directory_iterator it{ input_dir };
	int i = 0;
	while (it != boost::filesystem::directory_iterator{}) {

		// Get input / output paths
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path path = entry.path();
		//std::string mesh_id = in_p.filename().string();
		std::string mesh_path = path.string();

		pcl::PolygonMeshPtr mesh(boost::make_shared<pcl::PolygonMesh>());
		if (boost::filesystem::extension(path) == ".obj") {
			pcl::io::loadPolygonFileOBJ(mesh_path, *mesh);
		} else if (boost::filesystem::extension(path) == ".ply") {
			pcl::io::loadPLYFile(mesh_path, *mesh);
		} else {
			printf("skipping file with extension: %s\n", boost::filesystem::extension(path).c_str());
			continue;
		}
		printf("loading PolygonMesh %i\n", i++);


		meshes.push_back(mesh);
		mesh_ids.push_back(mesh_path);
	}
}

/* loads all TextureMeshes (saved as .obj files) from a dir (assumes every file is a .obj file)
 *
 */
void load_meshes_from_dir(const std::string dir_name, std::vector<pcl::TextureMeshPtr> &meshes, std::vector<std::string> &mesh_ids) {
	boost::filesystem::path input_dir(dir_name);

	boost::filesystem::directory_iterator it{ input_dir };
	int i = 0;
	while (it != boost::filesystem::directory_iterator{}) {
		// Get input / output paths
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path path = entry.path();

		if (boost::filesystem::extension(path) != ".obj") {
			printf("skipping file with extension: %s\n", boost::filesystem::extension(path).c_str());
			continue;
		}
		printf("loading TextureMesh %i\n", i++);

		std::string mesh_path = path.string();

		pcl::TextureMeshPtr mesh(boost::make_shared<pcl::TextureMesh>());
		pcl::io::loadOBJFile(mesh_path, *mesh);		// this is broken for TextureMeshes with multiple materials, 
													// all texture coordinates get loaded into the first submesh 
													// (any other submeshes get no texture coordinates)

		//// quick hack for files with multiple materials -- this just uses material_0 for every submesh, since visualization only supports 1 material
		//// (this causes each submesh other than the first to look wrong)
		//pcl::io::loadPolygonFileOBJ(mesh_path, *mesh);
		//pcl::TextureMesh mesh2;
		//pcl::io::loadOBJFile(mesh_path, mesh2);
		//mesh->tex_materials.clear();
		//mesh->tex_materials.push_back(mesh2.tex_materials[0]);

		meshes.push_back(mesh);
		mesh_ids.push_back(mesh_path);
	}
}

///* loads all PolygonMeshes / TextureMeshes from a dir (assumes every file is a .obj / .ply file)
// * assumes PolygonMeshes are stored in .ply files
// * assumes TextureMeshes are stored in .obj files
// *
// * input:
// *   - template type T: PolygonMeshPtr or TextureMeshPtr
// */
//template <class T>
//void load_meshes_from_dir(const std::string dir_name, std::vector<T> &meshes, std::vector<std::string> &mesh_ids) {
//	boost::filesystem::path input_dir(dir_name);
//
//	boost::filesystem::directory_iterator it{ input_dir };
//	int i = 0;
//	while (it != boost::filesystem::directory_iterator{}) {
//		printf("loading mesh %i\n", i++);
//
//		// Get input / output paths
//		boost::filesystem::directory_entry entry = *it++;
//		boost::filesystem::path in_p = entry.path();
//		std::string mesh_path = in_p.string();
//
//		T mesh;
//		if (std::is_same<T, pcl::TextureMeshPtr>::value) {
//			pcl::io::loadOBJFile(mesh_path, *mesh);
//		}
//		else if (std::is_same<T, pcl::PolygonMeshPtr>::value) {
//			pcl::io::loadPLYFile(mesh_path, *mesh);
//		}
//		pcl::io::loadOBJFile(mesh_path, *mesh);
//
//		meshes.push_back(mesh);
//		mesh_ids.push_back(mesh_path);
//		
//	}
//}

/* loads all clouds in `dir_name` and stores them in `clouds`. assumes each file is a .ply point cloud
*/
void load_clouds_from_dir(const std::string dir_name, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds, std::vector<std::string> &ids) {
	boost::filesystem::path input_dir(dir_name);

	boost::filesystem::directory_iterator it{ input_dir };
	int i = 0;
	while (it != boost::filesystem::directory_iterator{}) {
		// Get input / output paths
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path in_p = entry.path();

		// skip if entry is a directory
		if (!boost::filesystem::is_regular_file(in_p))
			continue;

		printf("loading cloud %i\n", i++);

		std::string cloud_id = in_p.string();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::io::loadPLYFile(cloud_id, *cloud);

		clouds.push_back(cloud);
		ids.push_back(cloud_id);
	}
}


/* Visualize a single mesh seq
 * (method takes type as an arg instead of being templated bc we are unable to easily cast templates in c++, 
 *  and the PCL methods themselves are not templated, requiring different function calls dependent on type)
 *
 * inputs:
 *   - // template type T: PolygonMeshPtr or TextureMeshPtr
 *   - dir_name:
 *   - type: "PolygonMesh" or "TextureMesh" are valid
*/
void viz_mesh_seq(const std::string dir_name, const std::string type = "PolygonMesh") {
	// Load meshes
	std::vector<pcl::PolygonMeshPtr> p_meshes;
	std::vector<pcl::TextureMeshPtr> t_meshes;
	std::vector<std::string> mesh_ids;

	if (type == "PolygonMesh") {
		load_meshes_from_dir(dir_name, p_meshes, mesh_ids);
	}
	else if (type == "TextureMesh") {
		load_meshes_from_dir(dir_name, t_meshes, mesh_ids);
	}

	// Setup visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();

	// this line is added to remove VTK console warnings, which slowed down the visualization
	// however, this doesn't work in vs2017 -- instead, we're using a custom `pcl_visualization_release.dll` which makes this change
	//
	// viewer->getRenderWindow()->GlobalWarningDisplayOff(); // doesn't work in vs2017? --edited 

	viewer->setCameraPosition(-236.42, -82.3299, -90.4387, -8.27996, -87.1787, 15.629, -0.0378362, -0.998645, 0.0357294);
	viewer->setCameraFieldOfView(0.8574994601);  // specified in radians, pressing 'c' on the vis app gives it in degrees (49.1311)
	viewer->setCameraClipDistances(149.222, 336.778);
	viewer->setPosition(0, 0);
	viewer->setSize(1173, 732);

	// Loop through meshes to display animation
	int mesh_idx = 0;
	while (!viewer->wasStopped()) {
		// ADD meshes
		if (type == "PolygonMesh") {
			viewer->addPolygonMesh(*p_meshes[mesh_idx], mesh_ids[mesh_idx], 0);
		} else if (type == "TextureMesh") {
			viewer->addTextureMesh(*t_meshes[mesh_idx], mesh_ids[mesh_idx], 0);
		}

		viewer->spinOnce(70);  // affects speed (time in ms each frame is shown for)
		//viewer->spinOnce(33);  // ~ 24 fps (1000 / 30fps = 33.33) kinect captures between 15-30fps depending on lighting? not sure
		//viewer->spinOnce();

		// REMOVE meshes
		viewer->removePolygonMesh(mesh_ids[mesh_idx]);
		if (mesh_idx == mesh_ids.size() - 1) {
			mesh_idx = 0;
		} else {
			mesh_idx++;
		}
	}
}

/* Visualize two seqs side by side
 *
 * inputs:
 *  - dir_name1:
 *  - dir_name2:
 *  - title_1: caption for left visualization
 *  - title_2: caption for right visualization
 *  - type1: "mesh" or "cloud" are only valid types
 *  - type2: "mesh" or "cloud" are only valid types
 */
void viz_seq_dual(const std::string dir_name1, const std::string dir_name2, const std::string title_1, const std::string title_2, const std::string type1="mesh", const std::string type2="mesh") {
	// Load meshes / point clouds
	std::vector<pcl::PolygonMeshPtr> meshes_1;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_1;
	std::vector<std::string> ids_1;
	int size_1;

	std::vector<pcl::PolygonMeshPtr> meshes_2;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_2;
	std::vector<std::string> ids_2;
	int size_2;

	if (type1 == "mesh") {
		load_meshes_from_dir(dir_name1, meshes_1, ids_1);
		size_1 = meshes_1.size();
	} else if (type1 == "cloud") {
		load_clouds_from_dir(dir_name1, clouds_1, ids_1);
		size_1 = clouds_1.size();
	} else {
		// error
	}

	if (type2 == "mesh") {
		load_meshes_from_dir(dir_name2, meshes_2, ids_2);
		size_2 = meshes_2.size();
	}
	else if (type2 == "cloud") {
		load_clouds_from_dir(dir_name2, clouds_2, ids_2);
		size_2 = clouds_2.size();
	}
	else {
		// error
	}

	if (size_1 != size_2) {
		// error
	}
	
	// Setup visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setPosition(0, 0);
	viewer->setSize(1173, 732);

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->addText(title_1, 10, 10, "v1 text", v1);
	viewer->setBackgroundColor(0, 1, 0, v1);
	viewer->setCameraPosition(-236.42, -82.3299, -90.4387, -8.27996, -87.1787, 15.629, -0.0378362, -0.998645, 0.0357294, v1);
	viewer->setCameraFieldOfView(0.8574994601, v1);  // specified in radians, pressing 'c' on the vis app gives it in degrees (49.1311)
	viewer->setCameraClipDistances(149.222, 336.778, v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->addText(title_2, 10, 10, "v2 text", v2);
	viewer->setBackgroundColor(1, 0, 0, v2);
	viewer->setCameraPosition(-236.42, -82.3299, -90.4387, -8.27996, -87.1787, 15.629, -0.0378362, -0.998645, 0.0357294, v2);
	viewer->setCameraFieldOfView(0.8574994601, v2);  // specified in radians, pressing 'c' on the vis app gives it in degrees (49.1311)
	viewer->setCameraClipDistances(149.222, 336.778, v2);

	// Loop through meshes to display animation
	int idx = 0;
	while (!viewer->wasStopped()) {
		if (type1 == "mesh") {
			viewer->addPolygonMesh(*meshes_1[idx], ids_1[idx], v1);
		} else {
			viewer->addPointCloud<pcl::PointXYZRGB>(clouds_1[idx], ids_1[idx], v1);
		}

		if (type2 == "mesh") {
			viewer->addPolygonMesh(*meshes_2[idx], ids_2[idx], v2);
		} else {
			viewer->addPointCloud<pcl::PointXYZRGB>(clouds_2[idx], ids_2[idx], v2);
		}

		viewer->spinOnce(70);  // affects speed (time in ms each frame is shown for)
		//viewer->spinOnce(33);  // ~ 24 fps (1000 / 30fps = 33.33) kinect captures between 15-30fps depending on lighting? not sure
		//viewer->spinOnce();

		if (type1 == "mesh") {
			viewer->removePolygonMesh(ids_1[idx], v1);
		}
		else {
			viewer->removePointCloud(ids_1[idx], v1);
		}

		if (type2 == "mesh") {
			viewer->removePolygonMesh(ids_2[idx], v2);
		}
		else {
			viewer->removePointCloud(ids_2[idx], v2);
		}

		if (idx == size_1 - 1) {
			idx = 0;
		}
		else {
			idx++;
		}
	}
}

/* Visualize two static point clouds side by side
*/
void viz_pc_dual(const std::string path1, const std::string path2, const std::string title_1, const std::string title_2) {

	// Load point clouds
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPLYFile(path1, *cloud1);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPLYFile(path2, *cloud2);

	// Setup visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setPosition(0, 0);
	viewer->setSize(1173, 732);

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->addText(title_1, 10, 10, "v1 text", v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setCameraPosition(-236.42, -82.3299, -90.4387, -8.27996, -87.1787, 15.629, -0.0378362, -0.998645, 0.0357294, v1);
	viewer->setCameraFieldOfView(0.8574994601, v1);  // specified in radians, pressing 'c' on the vis app gives it in degrees (49.1311)
	viewer->setCameraClipDistances(149.222, 336.778, v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->addText(title_2, 10, 10, "v2 text", v2);
	viewer->setBackgroundColor(0, 0, 0, v2);
	viewer->setCameraPosition(-236.42, -82.3299, -90.4387, -8.27996, -87.1787, 15.629, -0.0378362, -0.998645, 0.0357294, v2);
	viewer->setCameraFieldOfView(0.8574994601, v2);  // specified in radians, pressing 'c' on the vis app gives it in degrees (49.1311)
	viewer->setCameraClipDistances(149.222, 336.778, v2);
	
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud1, "cloud1", v1);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, "cloud2", v2);

	viewer->spin();
}

//void viz_mesh_seq_triple();

void viz_single_obj(const std::string obj_path) {

	// Setup visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->setPosition(0, 0);
	//viewer->setSize(1920, 1080);

	//viewer->setCameraPosition(-236.42, -82.3299, -90.4387, -8.27996, -87.1787, 15.629, -0.0378362, -0.998645, 0.0357294); 
	//viewer->setCameraPosition(8.436389419, -103.2375178, -270.2602032, -8.27996, -87.1787, 15.629, -0.0378362, -0.998645, 0.0357294); // manually positioned / oriented camera
	
	// "50_04" cam
	//viewer->setCameraPosition(224.205, -104.328, 141.558, -8.27996, -87.1787, 15.629, -0.0378362, -0.998645, 0.0357294); // "50_04" manually oriented cam
	//viewer->setCameraPosition(224.205, -104.328, 141.558, 0.877666245, -0.0170003581, -0.4789707197, -0.01219016786, 0.9982555568, -0.05776887742); // z column, y column
	//viewer->setCameraPosition(224.205, -104.328, 141.558, 0.877666245, -0.0170003581, -0.4789707197, 0.01219016786, -0.9982555568, 0.05776887742); // z column, y column inverse
	//viewer->setCameraPosition(224.205, -104.328, 141.558, -0.8759279688, -0.05776887742, -0.4789707197, -0.0565405, 0.998256, -0.0170004); // z row, y row
	viewer->setCameraPosition(224.205, -104.328, 141.558, 223.329, -104.386, 141.079, 0.0565405, -0.998256, 0.0170004); // pos + z row, y row inverse
	
	// "50_01" cam
	//viewer->setCameraPosition(-109.374, -105.256, -246.431, -8.27996, -87.1787, 15.629, -0.0378362, -0.998645, 0.0357294); // "50_01" manually oriented cam
	//viewer->setCameraPosition(-109.374, -105.256, -246.431, 0.434116, 0.00748438, 0.900826, -0.0378362, -0.998645, 0.0357294); // z "in" axis
	//viewer->setCameraPosition(-109.374, -105.256, -246.431, -0.00339385, 0.999972, -0.00667259, -0.0378362, -0.998645, 0.0357294); // y "in" axis
	//viewer->setCameraPosition(-109.374, -105.256, -246.431, 0.900851, 0.000160591, -0.434129, -0.0378362, -0.998645, 0.0357294); // x "in" axis

	//viewer->setCameraFieldOfView(0.8574994601);  // specified in radians, pressing 'c' on the vis app gives it in degrees (49.1311)
	//viewer->setCameraClipDistances(149.222, 336.778);

	double fovy = 2 * atan(1080 / (2. * 1052.37)) ;
	//fovy *= M_PI / 180.0;
	viewer->setCameraFieldOfView(fovy);
	viewer->setCameraClipDistances(0.01, 1000.01);

	//// try setting directly from extrinsics / intrinsics matricies -- doesnt work
	//Eigen::Matrix4f extrinsics;
	////				5, 6, 7, 8,
	////				9, 10, 11, 12,
	////				13, 14, 15, 16;
	//extrinsics <<	-0.479117274, -0.01219016786, 0.877666245, -18.09196571,
	//				-0.0565405272, 0.9982555568, -0.0170003581, 119.2292113,
	//				-0.8759279688, -0.05776887742, -0.4789707197, 258.1626559,
	//				0, 0, 0, 1;
	//Eigen::Matrix3f intrinsics;
	//intrinsics <<	1052.37, 0, 953.062, 
	//				0, 1052.37, 535.367, 
	//				0, 0, 1;
	//viewer->setCameraParameters(intrinsics, extrinsics);

	// load TextureMesh from .obj file
	pcl::TextureMeshPtr mesh(boost::make_shared<pcl::TextureMesh>());
	int err = pcl::io::loadOBJFile(obj_path, *mesh);	// material file path (specified in .obj) must be local 
														// tex file (specified in .mtl) can be absolute (and local maybe?)

	viewer->addTextureMesh(*mesh, "mesh");  // .mtl texture file must be an absolute path!

	//viewer->spinOnce(1, true);
	//viewer->saveScreenshot("C:/Users/maxhu/Desktop/uvatlas_example/test_panoptic_screenshot.png");
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

int main(int argc, char** argv) {

	//// visualize single mesh seq
	//std::string dir_name = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/pcl_mesh/decimated_0.900000";
	//viz_mesh_seq(dir_name, "PolygonMesh");

	//// SPSR on MLS smoothed points vs non-MLS smoothed points (90% decimated)
	//std::string dir_name1 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/pcl_mesh/decimated_0.900000";
	//std::string dir_name2 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_decimated_0.900000";
	//viz_seq_dual(dir_name1, dir_name2, "Reconstruction without MLS", "Reconstruction with MLS");

	//// MLS smoothed PC vs non-MLS smoothed PC
	//std::string path1 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/normals_cleaned/ptcloud_hd00000380_normals_cleaned.ply";
	//std::string path2 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/ptcloud_hd00000380_normals_cleaned.ply";
	//viz_pc_dual(path1, path2, "no MLS smoothing", "with MLS smoothing");

	//// Full SPSR vs 90% decimation SPSR (on mls smoothed pc)
	//std::string dir_name1 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_full";
	//std::string dir_name2 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_decimated_0.900000";
	//viz_seq_dual(dir_name1, dir_name2, "No decimation", "90% decimation");

	//// PC on left, reconstruction on right
	//std::string dir_name1 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls";
	//std::string dir_name2 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_full";
	//viz_seq_dual(dir_name1, dir_name2, "", "", "cloud", "mesh");

	////// seq length 1 test
	//std::string dir_name = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/single_test";
	//viz_mesh_seq(dir_name, "PolygonMesh");


	//// visualize single obj mesh with tex
	//// material file path (specified in .obj) must be local
	//// tex file path (specified in.mtl) must be absolute
	//std::string obj_path = "C:/Users/maxhu/Desktop/uvatlas_example/cube.obj";  // no material (access violation error)
	//std::string obj_path = "C:/Users/maxhu/Desktop/uvatlas_example/test_cube.obj";  // single mat with .jpg tex (local .mtl path, absolute tex path) --works!!
	//std::string obj_path = "C:/Users/maxhu/Desktop/uvatlas_example/test_panoptic.obj";  // single mat with .bmp tex (absolute path) (red)
	std::string obj_path = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_texture_mapped/ptcloud_hd00000380_normals_cleaned.obj";  // (local .mtl path, absolute tex path) (red)
	//std::string obj_path = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient/bmps/ptcloud_hd00000380_normals_cleaned.obj"; // (local .mtl path, absolute tex path) (red)
	//std::string obj_path = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient/pngs/ptcloud_hd00000380_normals_cleaned.obj"; // (local .mtl path, absolute tex path) (weird black lines)
	//std::string obj_path = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient/jpgs/ptcloud_hd00000380_normals_cleaned.obj"; // (local .mtl path, absolute tex path) (weird black lines)
	viz_single_obj(obj_path);

	//// visualize TextureMesh
	//std::string dir_name = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh";
	//viz_mesh_seq(dir_name, "TextureMesh");
	////viz_mesh_seq(dir_name, "PolygonMesh");

	//// visualize UV gradient Texture
	//std::string dir_name = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uv_gradient";
	//viz_mesh_seq(dir_name, "TextureMesh");
}