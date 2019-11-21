#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <boost/filesystem.hpp>
#include <vector>
#include <stdio.h>

int main(int argc, char** argv)
{
	std::vector<pcl::PolygonMeshPtr> meshes;
	std::vector<std::string> mesh_ids;

	// Load meshes
	boost::filesystem::path input_dir("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/pcl_mesh/decimated_0.900000");
	boost::filesystem::directory_iterator it{ input_dir };
	int i = 0;
	while (it != boost::filesystem::directory_iterator{}) {
		printf("loading mesh %i\n", i++);

		// Get input / output paths
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path in_p = entry.path();
		std::string mesh_id = in_p.filename().string();

		pcl::PolygonMeshPtr mesh(boost::make_shared<pcl::PolygonMesh>());
		pcl::io::loadPLYFile(in_p.string(), *mesh);

		meshes.push_back(mesh);
		mesh_ids.push_back(mesh_id);
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
		viewer->addPolygonMesh(*meshes[mesh_idx], mesh_ids[mesh_idx], 0);

		viewer->spinOnce(70);  // affects speed (time in ms each frame is shown for)
		//viewer->spinOnce(33);  // ~ 24 fps (1000 / 30fps = 33.33) kinect captures between 15-30fps depending on lighting? not sure
		//viewer->spinOnce();

		viewer->removePolygonMesh(mesh_ids[mesh_idx]);
		if (mesh_idx == meshes.size() - 1) {
			mesh_idx = 0;
		} else {
			mesh_idx++;
		}
	}

}