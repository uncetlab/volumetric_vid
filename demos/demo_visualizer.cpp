#include <volcap/viz/viz.h>
#include <string>

// get the project directory from preprocessor set in the top-level CMakeLists.txt
const std::string PROJECT_DIR = _PROJECT_DIR;

int main(int argc, char** argv) {
	// uncomment a section to try out a visualization

	std::string dir_name, dir_name1, dir_name2;

	//// visualize single mesh seq
	//dir_name = PROJECT_DIR + "/demos/demo_output/03_mesh_decimated/decimated_0.800000";
	//volcap::viz::viz_mesh_seq(dir_name, "PolygonMesh");

	//// SPSR on MLS smoothed points vs non-MLS smoothed points (90% decimated) // TODO: not generating meshes reconstructed on unsmoothed points at the moment
	//dir_name1 = "?";
	//dir_name2 = PROJECT_DIR + "/demos/demo_output/03_mesh_decimated/decimated_0.800000";
	//volcap::viz::viz_seq_dual(dir_name1, dir_name2, "Reconstruction without MLS", "Reconstruction with MLS");

	//// MLS smoothed PC vs non-MLS smoothed PC
	//std::string path1 = PROJECT_DIR + "/demo_data/kinoptic_ptclouds/ptcloud_hd00000380.ply";
	//std::string path2 = PROJECT_DIR + "/demos/demo_output/01_cloud_mls/ptcloud_hd00000380.ply";
	//volcap::viz::viz_pc_dual(path1, path2, "no MLS smoothing", "with MLS smoothing");

	//// Full SPSR vs 80% decimation SPSR (on mls smoothed pc)
	//dir_name1 = PROJECT_DIR + "/demos/demo_output/02_mesh_spsr";
	//dir_name2 = PROJECT_DIR + "/demos/demo_output/03_mesh_decimated/decimated_0.800000";
	//volcap::viz::viz_seq_dual(dir_name1, dir_name2, "No decimation", "80% decimation");

	// PC on left, reconstruction on right
	dir_name1 = PROJECT_DIR + "/demos/demo_output/01_cloud_mls";
	dir_name2 = PROJECT_DIR + "/demos/demo_output/02_mesh_spsr";
	volcap::viz::viz_seq_dual(dir_name1, dir_name2, "", "", "cloud", "mesh");

	//// seq length 1 test
	//std::string dir_name = "";
	//volcap::viz::viz_mesh_seq(dir_name, "PolygonMesh");


	/*********************************************************************************************************************/
	// TODO: visualizing TextureMeshes is bugged, doesn't work with .bmps, kind of works with .jpg / .png, but poor
	/*********************************************************************************************************************/

	//// visualize single obj mesh with tex	
	//// material file path (specified in .obj) must be local
	//// tex file path (specified in.mtl) must be absolute
	////std::string obj_path = "C:/Users/maxhu/Desktop/uvatlas_example/cube.obj";  // no material (access violation error)
	////std::string obj_path = "C:/Users/maxhu/Desktop/uvatlas_example/test_cube.obj";  // single mat with .jpg tex (local .mtl path, absolute tex path) --works!!
	////std::string obj_path = "C:/Users/maxhu/Desktop/uvatlas_example/test_panoptic.obj";  // single mat with .bmp tex (absolute path) (red)
	////std::string obj_path = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_texture_mapped/ptcloud_hd00000380_normals_cleaned.obj";  // (local .mtl path, absolute tex path, bmp) (red)
	//std::string obj_path = PROJECT_DIR + "/demos/demo_output/05_mesh_textured/ptcloud_hd00000380.obj"; // (local .mtl path, absolute tex path, bmp) (red)
	////std::string obj_path = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient/pngs/ptcloud_hd00000380_normals_cleaned.obj"; // (local .mtl path, absolute tex path, png) (weird black lines)
	////std::string obj_path = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient/jpgs/ptcloud_hd00000380_normals_cleaned.obj"; // (local .mtl path, absolute tex path, jpg) (weird black lines)
	//volcap::viz::viz_single_obj(obj_path);

	//// visualize TextureMesh -- (texture appears red)
	//std::string dir_name = PROJECT_DIR + "/demos/demo_output/05_mesh_textured";
	//volcap::viz::viz_mesh_seq(dir_name, "TextureMesh");
	////volcap::viz::viz_mesh_seq(dir_name, "PolygonMesh");

	//// visualize UV gradient Texture -- unhandled exception
	//std::string dir_name = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uv_gradient";
	//volcap::viz::viz_mesh_seq(dir_name, "TextureMesh");
}
