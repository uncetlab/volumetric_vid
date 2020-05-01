#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <volcap/io/io.h>
#include <volcap/viz/viz.h>

void volcap::viz::viz_mesh_seq(const std::string dir_name, const std::string type) {
	// Load meshes
	std::vector<pcl::PolygonMeshPtr> p_meshes;
	std::vector<pcl::TextureMeshPtr> t_meshes;
	std::vector<std::string> mesh_ids;

	if (type == "PolygonMesh") {
		volcap::io::load_meshes_from_dir(dir_name, p_meshes, mesh_ids);
	} else if (type == "TextureMesh") {
		volcap::io::load_meshes_from_dir(dir_name, t_meshes, mesh_ids);
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

void volcap::viz::viz_seq_dual(const std::string dir_name1, const std::string dir_name2, const std::string title_1, const std::string title_2, const std::string type1, const std::string type2) {
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
		volcap::io::load_meshes_from_dir(dir_name1, meshes_1, ids_1);
		size_1 = meshes_1.size();
	} else if (type1 == "cloud") {
		volcap::io::load_clouds_from_dir<pcl::PointXYZRGB>(dir_name1, clouds_1, ids_1);
		size_1 = clouds_1.size();
	} else {
		// error
	}

	if (type2 == "mesh") {
		volcap::io::load_meshes_from_dir(dir_name2, meshes_2, ids_2);
		size_2 = meshes_2.size();
	} else if (type2 == "cloud") {
		volcap::io::load_clouds_from_dir<pcl::PointXYZRGB>(dir_name2, clouds_2, ids_2);
		size_2 = clouds_2.size();
	} else {
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
		} else {
			viewer->removePointCloud(ids_1[idx], v1);
		}

		if (type2 == "mesh") {
			viewer->removePolygonMesh(ids_2[idx], v2);
		} else {
			viewer->removePointCloud(ids_2[idx], v2);
		}

		if (idx == size_1 - 1) {
			idx = 0;
		} else {
			idx++;
		}
	}
}

void volcap::viz::viz_pc_dual(const std::string path1, const std::string path2, const std::string title_1, const std::string title_2) {

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

	viewer->addPointCloud<pcl::PointXYZRGB>(cloud1, "cloud1", v1);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud2, "cloud2", v2);

	viewer->spin();
}

void volcap::viz::viz_single_obj(const std::string obj_path) {

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

	double fovy = 2 * atan(1080 / (2. * 1052.37));
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
