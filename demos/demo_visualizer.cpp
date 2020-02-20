#include <volcap/viz/viz.h>

int main(int argc, char** argv) {

	//// visualize single mesh seq
	//std::string dir_name = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/pcl_mesh/decimated_0.900000";
	//volcap::viz::viz_mesh_seq(dir_name, "PolygonMesh");

	//// SPSR on MLS smoothed points vs non-MLS smoothed points (90% decimated)
	//std::string dir_name1 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/pcl_mesh/decimated_0.900000";
	//std::string dir_name2 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_decimated_0.900000";
	//volcap::viz::viz_seq_dual(dir_name1, dir_name2, "Reconstruction without MLS", "Reconstruction with MLS");

	//// MLS smoothed PC vs non-MLS smoothed PC
	//std::string path1 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/normals_cleaned/ptcloud_hd00000380_normals_cleaned.ply";
	//std::string path2 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/ptcloud_hd00000380_normals_cleaned.ply";
	//volcap::viz::viz_pc_dual(path1, path2, "no MLS smoothing", "with MLS smoothing");

	//// Full SPSR vs 90% decimation SPSR (on mls smoothed pc)
	//std::string dir_name1 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_full";
	//std::string dir_name2 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_decimated_0.900000";
	//volcap::viz::viz_seq_dual(dir_name1, dir_name2, "No decimation", "90% decimation");

	//// PC on left, reconstruction on right
	//std::string dir_name1 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls";
	//std::string dir_name2 = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_full";
	//volcap::viz::viz_seq_dual(dir_name1, dir_name2, "", "", "cloud", "mesh");

	//// seq length 1 test
	//std::string dir_name = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/single_test";
	//volcap::viz::viz_mesh_seq(dir_name, "PolygonMesh");


	//// visualize single obj mesh with tex
	//// material file path (specified in .obj) must be local
	//// tex file path (specified in.mtl) must be absolute
	//std::string obj_path = "C:/Users/maxhu/Desktop/uvatlas_example/cube.obj";  // no material (access violation error)
	//std::string obj_path = "C:/Users/maxhu/Desktop/uvatlas_example/test_cube.obj";  // single mat with .jpg tex (local .mtl path, absolute tex path) --works!!
	//std::string obj_path = "C:/Users/maxhu/Desktop/uvatlas_example/test_panoptic.obj";  // single mat with .bmp tex (absolute path) (red)
	//std::string obj_path = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_texture_mapped/ptcloud_hd00000380_normals_cleaned.obj";  // (local .mtl path, absolute tex path, bmp) (red)
	//std::string obj_path = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient/bmps/ptcloud_hd00000380_normals_cleaned.obj"; // (local .mtl path, absolute tex path, bmp) (red)
	//std::string obj_path = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient/pngs/ptcloud_hd00000380_normals_cleaned.obj"; // (local .mtl path, absolute tex path, png) (weird black lines)
	std::string obj_path = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient/jpgs/ptcloud_hd00000380_normals_cleaned.obj"; // (local .mtl path, absolute tex path, jpg) (weird black lines)
	volcap::viz::viz_single_obj(obj_path);

	//// visualize TextureMesh -- access violation
	//std::string dir_name = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh";
	//volcap::viz::viz_mesh_seq(dir_name, "TextureMesh");
	////volcap::viz::viz_mesh_seq(dir_name, "PolygonMesh");

	//// visualize UV gradient Texture -- unhandled exception
	//std::string dir_name = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uv_gradient";
	//volcap::viz::viz_mesh_seq(dir_name, "TextureMesh");
}