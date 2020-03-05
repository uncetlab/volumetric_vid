#include <pcl/surface/texture_mapping.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <volcap/io/io.h>
#include <volcap/io/io_cam.h>
#include <volcap/texture/texturing.h>

// get the project directory from preprocessor set in the top-level CMakeLists.txt
const std::string PROJECT_DIR = _PROJECT_DIR;

//! Display a 3D representation showing the cloud and a list of camera with their 6DOf poses
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
		} else {
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

/** 
 * custom segmentation + texturing demo
 * uses UVAtlas' uv-mapping + original images to create a single texture file for the mesh
 * output is currently just a .bmp texture map, you must manually set the tex in material file to it
 */
void custom_seg_demo() {
	volcap::texture::Texturing t;

	//==> load TextureMesh with UVAtlas' uv-mapping
	pcl::TextureMesh tmesh;
	pcl::io::loadOBJFile("C:/Users/maxhu/Desktop/uvatlas_example/texture_mapping_tests/test_panoptic.obj", tmesh);

	//==> load cameras
	pcl::texture_mapping::CameraVector my_cams;
	volcap::io::loadCameraParams("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/calibration_171026_pose3.json", my_cams);

	//// or specify just a single cam
	//std::vector<std::string> cams_to_use;  // list of camera names to get loaded
	//cams_to_use.push_back("50_02");
	//loadCameraParams("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/calibration_171026_pose3.json", my_cams, &cams_to_use);

	//==> segment using custom func
	std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> tex_coords;
	std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> img_coords;
	t.segmentUVMeshByCamera(tmesh, my_cams, tex_coords, img_coords);

	//==> prepare image files
	std::vector<std::string> img_files;
	for (int cam_idx = 0; cam_idx < my_cams.size(); cam_idx++) {
		img_files.push_back(my_cams[cam_idx].texture_file);
	}

	//==> generate texture map using UVAtlas' uv-map, greedy custom segmentation
	//std::string texture_file_name = "C:/Users/maxhu/Desktop/uvatlas_example/texture_mapping_tests/test_panoptic_texture_all.bmp";
	//std::string texture_file_name = "C:/Users/maxhu/Desktop/uvatlas_example/texture_mapping_tests/test_panoptic_texture_04.bmp";
	//std::string texture_file_name = "C:/Users/maxhu/Desktop/uvatlas_example/texture_mapping_tests/kd-tree_occlusion_50_02.bmp";
	std::string texture_file_name = "C:/Users/maxhu/Desktop/uvatlas_example/texture_mapping_tests/kd-tree_occlusion_all.bmp";
	t.generateUVTextureFromImages(texture_file_name, tex_coords, img_coords, img_files);
}

// custom segmentation + texturing demo for a sequence of meshes
void custom_seg_dir_demo(std::string input_dir, std::string output_dir, std::string calibration_file) {
	volcap::texture::Texturing t;
	boost::filesystem::create_directory(output_dir);

	//==> load uv-mapped texture meshes
	std::vector<pcl::TextureMeshPtr> meshes;
	std::vector<std::string> mesh_ids;
	volcap::io::load_meshes_from_dir(input_dir, meshes, mesh_ids);

	//==> load cameras
	pcl::texture_mapping::CameraVector my_cams;		// note we don't use the 'texture_file' member of the cams in our funcs, so the cams are reused for
													// every frame. we prepare the texture files for each frame using `img_files` vector
	volcap::io::loadCameraParams(calibration_file, my_cams);

	int seq_start_frame = 380;  // hardcoded for demo!
	for (int idx_mesh = 0; idx_mesh < meshes.size(); idx_mesh++, seq_start_frame++) {
		pcl::TextureMesh &mesh = *meshes[idx_mesh];

		//==> segment using custom func
		std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> tex_coords;
		std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> img_coords;
		t.segmentUVMeshByCamera(mesh, my_cams, tex_coords, img_coords);

		//==> prepare image files
		std::vector<std::string> img_files;
		for (int idx_cam = 0; idx_cam < my_cams.size(); idx_cam++) {
			//50_02_00000001.jpg
			boost::filesystem::path tex_file_path(my_cams[idx_cam].texture_file);
			std::string tex_filename = tex_file_path.filename().string();
			std::string cam_name = tex_filename.substr(0, 5);  // assume cam name is first 5 chars of texture_file
			char file_name[19];  // file name lengths are all 18, use 19 to prevent mem error
			sprintf(file_name, "%s_%08i.jpg", cam_name.c_str(), seq_start_frame);
			std::string file_name_str(file_name);

			std::string parent_path = tex_file_path.parent_path().string();
			std::string full_path = parent_path + "/" + file_name_str;
			img_files.push_back(full_path);
		}

		//==> generate texture map using UVAtlas' uv-map, greedy custom segmentation
		std::string texture_file = mesh_ids[idx_mesh] + ".bmp";
		std::string texture_file_full = output_dir + "/" + texture_file;
		t.generateUVTextureFromImages(texture_file_full, tex_coords, img_coords, img_files);

		//==> update TextureMesh material to use new texture file
		//mesh.tex_materials[0].tex_file = texture_file_full;  // saving full path in texture material
		mesh.tex_materials[0].tex_file = texture_file;  // saving relative path in texture material (preferred if it works)

		//==> resave TextureMesh as .obj
		std::string obj_path = output_dir + "/" + mesh_ids[idx_mesh] + ".obj";  // path must be declared with forward slashes to work correctly
		pcl::io::saveOBJFile(obj_path, mesh);
	}
}

/**
 * -PCL's textureMeshwithMultipleCameras() demo
 * -textureMeshwithMultipleCameras does camera segmentation and the uv mapping for each camera
 * -Uses separate materials for each camera (tex files are the original images)
 * -Less favorable than our custom method, which creates one single texture file for the mesh
 */
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

	// load all cams
	volcap::io::loadCameraParams("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/calibration_171026_pose3.json", my_cams);

	//// or specify just a single cam
	//std::vector<std::string> cams_to_use;  // list of camera names to get loaded
	//cams_to_use.push_back("50_02");
	//volcap::io::loadCameraParams("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/calibration_171026_pose3.json", my_cams, &cams_to_use);

	//// or just manually set one camera
	//pcl::TextureMapping<pcl::PointXYZ>::Camera &cam;
	//volcap::io::hardcodeLoadCameraParam(cam);
	//my_cams.push_back(cam);

	//====> Display cameras to user
	PCL_INFO("\nDisplaying cameras. Press \'q\' to continue texture mapping\n");
	showCameras(my_cams, cloud);

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
	//std::string file_name = "pcl_texturing_50_02.obj";
	std::string file_name = "pcl_texturing_all_cams.obj";
	std::string save_message = "\nSaving mesh to " + file_name + "\n";
	PCL_INFO(save_message.c_str());
	pcl::io::saveOBJFile("C:/Users/maxhu/Desktop/uvatlas_example/texture_mapping_tests/" + file_name, tmesh);  // MUST be declared with forward slashes to work correctly
}

/**
 * demos PCL's TextureMapping::mapTexture2MeshUV to create UVMapping
 * -this produces a bugged uv-map, some coordinates are > 1 ? and they appear to overlap
 */
void pcl_uvmap_demo() {
	volcap::texture::Texturing t;

	// ====== load geometry into a PolygonMesh
	pcl::PolygonMesh pmesh;
	std::string fname = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_decimated_0.900000/ptcloud_hd00000380_normals_cleaned.ply";
	pcl::io::loadPLYFile(fname, pmesh);

	// ====== transfer points / faces to a TextureMesh
	pcl::TextureMesh tmesh;
	tmesh.cloud = pmesh.cloud;
	std::vector< pcl::Vertices> tmesh_polygons;
	tmesh_polygons.resize(pmesh.polygons.size());

	for (size_t i = 0; i < pmesh.polygons.size(); ++i) {
		tmesh_polygons[i] = pmesh.polygons[i];
	}
	tmesh.tex_polygons.push_back(tmesh_polygons);

	// ====== create uv mapping
	pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
	pcl::TexMaterial tex_mat;
	tm.setTextureMaterials(tex_mat); // push null tex material so functions dont complain

	std::string out_tex_file = "C:/Users/maxhu/Desktop/uvatlas_example/texture_mapping_tests/pcl_uvmap_demo.bmp";
	std::vector<std::string> tex_files;
	//tex_files.push_back("C:/Users/maxhu/Desktop/uvatlas_example/occluded.jpg");
	tex_files.push_back(out_tex_file);
	tm.setTextureFiles(tex_files); // push dummy tex file too

	//tm.mapTexture2Mesh(tmesh);
	tm.mapTexture2MeshUV(tmesh);

	// ====== generate texture with uv mapping
	t.generateGradientTexture(out_tex_file, tmesh.tex_coordinates[0]);

	// ====== compute normals for the mesh so that saveOBJFile() doesn't complain
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pmesh.cloud, *cloud);

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

	pcl::toPCLPointCloud2(*cloud_with_normals, tmesh.cloud);

	// ====== write TextureMesh to .obj
	pcl::io::saveOBJFile("C:/Users/maxhu/Desktop/uvatlas_example/texture_mapping_tests/pcl_uvmap_demo.obj", tmesh);
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

	//// single test on test_panoptic.obj
	//custom_seg_demo();

	// texturing onto UVAtlas' uv-map
	boost::filesystem::path input_dir(PROJECT_DIR + "/demos/demo_output/04_mesh_uv-mapped");
	boost::filesystem::path out_dir(PROJECT_DIR + "/demos/demo_output/05_mesh_textured");
	boost::filesystem::path calibration_file(PROJECT_DIR + "/demo_data/calibration_171026_pose3.json");
	custom_seg_dir_demo(input_dir.string(), out_dir.string(), calibration_file.string());

	//// single test using PCL's textureMeshwithMultipleCameras()
	//pcl_texture_demo();

	//pcl_uvmap_demo();

	//pcl_segmentation_with_custom_texture_demo();

}