#include <pcl/visualization/pcl_visualizer.h>

template <typename PointT>
void volcap::viz::viz_pc (
	typename pcl::PointCloud<PointT>::Ptr cloud,
	std::vector<volcap::io::Camera*> cams
) {
	// Setup visualizer
	pcl::visualization::PCLVisualizer viewer("cameras");

	// add axis
	viewer.addCoordinateSystem(2.0);

	// add cloud
	viewer.addPointCloud<PointT>(cloud, "cloud");

	// add frustrums
	for (int i = 0; i < cams.size(); ++i) {
		volcap::io::Camera cam = *cams[i];
		Eigen::Affine3f pose = cam.getPose();

		double focal_h = cam.K(1, 1);
		double focal_w = cam.K(0, 0);
		double height = cam.height;
		double width = cam.width;

		float scale = 1.0; // 50.0

		// create a 5-point frustrum
		pcl::PointXYZ p1, p2, p3, p4, p5, location, view_dir;
		p1.x = 0; p1.y = 0; p1.z = 0;
		double dist = 0.75;
		double angleX, angleY;
		double minX, minY, maxX, maxY;

		maxX = dist * tan(atan(width / (2.0*focal_w)));
		maxY = dist * tan(atan(height / (2.0*focal_h)));
		angleX = RAD2DEG(2.0 * atan(width / (2.0*focal_w)));
		angleY = RAD2DEG(2.0 * atan(height / (2.0*focal_h)));

		minX = -maxX;
		minY = -maxY;

		p2.x = minX * scale; p2.y = minY * scale; p2.z = dist * scale;
		p3.x = maxX * scale; p3.y = minY * scale; p3.z = dist * scale;
		p4.x = maxX * scale; p4.y = maxY * scale; p4.z = dist * scale;
		p5.x = minX * scale; p5.y = maxY * scale; p5.z = dist * scale;
		p1 = pcl::transformPoint(p1, pose);
		p2 = pcl::transformPoint(p2, pose);
		p3 = pcl::transformPoint(p3, pose);
		p4 = pcl::transformPoint(p4, pose);
		p5 = pcl::transformPoint(p5, pose);

		std::stringstream ss;
		ss << cam.name;
		viewer.addText3D(ss.str(), p1, 0.1*scale, 1.0, 1.0, 1.0, ss.str());
		ss.str("");
		ss << "camera_" << i << "line1";
		viewer.addLine(p1, p2, ss.str());
		ss.str("");
		ss << "camera_" << i << "line2";
		viewer.addLine(p1, p3, ss.str());
		ss.str("");
		ss << "camera_" << i << "line3";
		viewer.addLine(p1, p4, ss.str());
		ss.str("");
		ss << "camera_" << i << "line4";
		viewer.addLine(p1, p5, ss.str());
		ss.str("");
		ss << "camera_" << i << "line5";
		viewer.addLine(p2, p5, ss.str());
		ss.str("");
		ss << "camera_" << i << "line6";
		viewer.addLine(p5, p4, ss.str());
		ss.str("");
		ss << "camera_" << i << "line7";
		viewer.addLine(p4, p3, ss.str());
		ss.str("");
		ss << "camera_" << i << "line8";
		viewer.addLine(p3, p2, ss.str());

		//// visualize the view vector
		//location.x = cam.pose(0, 3);
		//location.y = cam.pose(1, 3);
		//location.z = cam.pose(2, 3);

		//view_dir.x = cam.pose(0, 2) * scale + location.x;
		//view_dir.y = cam.pose(1, 2) * scale + location.y;
		//view_dir.z = cam.pose(2, 2) * scale + location.z;

		//ss.str("");
		//ss << "camera_" << i << "view_vec";
		//viewer.addLine(location, view_dir, 0.0, 0.0, 1.0, ss.str());
	}

	viewer.spin();
}
