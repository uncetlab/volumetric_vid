#include <pcl/surface/texture_mapping.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>

/** \brief Display a 3D representation showing the a cloud and a list of camera with their 6DOf poses */
void showCameras(pcl::texture_mapping::CameraVector cams, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

	// visualization object
	pcl::visualization::PCLVisualizer visu("cameras");

	// add a visual for each camera at the correct pose
	for (int i = 0; i < cams.size(); ++i)
	{
		// read current camera
		pcl::TextureMapping<pcl::PointXYZ>::Camera cam = cams[i];
		double focal = cam.focal_length;
		double height = cam.height;
		double width = cam.width;

		float scale = 50.0;
		// create a 5-point visual for each camera
		pcl::PointXYZ p1, p2, p3, p4, p5;
		p1.x = 0; p1.y = 0; p1.z = 0;
		double angleX = RAD2DEG(2.0 * atan(width / (2.0*focal)));
		double angleY = RAD2DEG(2.0 * atan(height / (2.0*focal)));
		double dist = 0.75;
		double minX, minY, maxX, maxY;
		maxX = dist * tan(atan(width / (2.0*focal)));
		minX = -maxX;
		maxY = dist * tan(atan(height / (2.0*focal)));
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

int main(int argc, char** argv) {

	// ==============================================================================================
	// load .ply mesh file 
	pcl::PolygonMesh pmesh;
	std::string fname = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_decimated_0.900000/ptcloud_hd00000380_normals_cleaned.ply";
	
	PCL_INFO("\nLoading mesh from file %s...\n", fname.c_str());
	pcl::io::loadPLYFile(fname, pmesh);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::fromROSMsg(pmesh.cloud, *cloud);
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


	// ==============================================================================================
	// Load textures and cameras poses and intrinsics
	PCL_INFO("\nLoading textures and camera poses...\n");
	pcl::texture_mapping::CameraVector my_cams;

	// TODO: automate camera loading like in example code below
	//const boost::filesystem::path base_dir(".");
	//std::string extension(".txt");
	//int cpt_cam = 0;
	//for (boost::filesystem::directory_iterator it(base_dir); it != boost::filesystem::directory_iterator(); ++it)
	//{
	//	if (boost::filesystem::is_regular_file(it->status()) && boost::filesystem::extension(it->path()) == extension)
	//	{
	//		pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
	//		readCamPoseFile(it->path().string(), cam);
	//		cam.texture_file = boost::filesystem::basename(it->path()) + ".png";
	//		my_cams.push_back(cam);
	//		cpt_cam++;
	//	}
	//}

	// manually set one camera
	pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
	cam.texture_file = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinectImgs/50_04/50_04_00000380.jpg";

	// location
	//cam.pose(0, 3) = -233.7954; //TX
	//cam.pose(1, 3) = -115.6553; //TY
	//cam.pose(2, 3) = 114.6928; //TZ

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
	cam.height = 1080;
	cam.width = 1920;
	cam.center_h = 535.367;
	cam.center_w = 953.062;

	
	my_cams.push_back(cam);

	PCL_INFO("\tLoaded %d textures.\n", my_cams.size());
	PCL_INFO("...Done.\n");

	// Display cameras to user
	PCL_INFO("\nDisplaying cameras. Press \'q\' to continue texture mapping\n");
	//showCameras(my_cams, cloud);

	// ==============================================================================================
	// Create materials for each texture (and one extra for occluded faces)
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

	// ==============================================================================================
	// Sort faces
	PCL_INFO("\nSorting faces by cameras...\n");
	pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
	tm.textureMeshwithMultipleCameras(tmesh, my_cams);

	PCL_INFO("Sorting faces by cameras done.\n");
	for (int i = 0; i <= my_cams.size(); ++i)
	{
		PCL_INFO("\tSub mesh %d contains %d faces and %d UV coordinates.\n", i, tmesh.tex_polygons[i].size(), tmesh.tex_coordinates[i].size());
	}

	// ==============================================================================================

	// compute normals for the mesh
	PCL_INFO("\nEstimating normals...\n");
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	// Concatenate XYZ and normal fields
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	PCL_INFO("...Done.\n");

	pcl::toPCLPointCloud2(*cloud_with_normals, tmesh.cloud);

	PCL_INFO("\nSaving mesh to textured_mesh.obj\n");

	// MUST be declared with forward slashes to work correctly
	pcl::io::saveOBJFile("C:/Users/maxhu/Desktop/uvatlas_example/test_texturing.obj", tmesh);
}