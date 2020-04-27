#include <volcap/io/io.h>
#include <volcap/io/io_cam.h>
#include <volcap/surface/spsr.h>
#include <volcap/texture/uv_map.h>
#include <volcap/texture/texturing.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <string>
#include <iostream>
#include <iterator>

namespace po = boost::program_options;

void reconstruction_pipeline(std::string in_dir, std::string out_dir, std::vector<volcap::io::Camera*> &cams) {

	// loop through clean point clouds
	//boost::filesystem::path in_dir_path(in_dir);
	//boost::filesystem::path out_dir_path(out_dir);
	boost::filesystem::path images_dir_path = boost::filesystem::path(in_dir) / "color";
	boost::filesystem::create_directories(out_dir);

	// 0. load in the pointclouds from in_dir
	//std::vector<pcl::TextureMeshPtr> meshes;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> clouds;
	std::vector<std::string> cloud_filenames;
	volcap::io::load_clouds_from_dir<pcl::PointNormal>(in_dir + "/pointcloud_merged", clouds, cloud_filenames, 2);  // these wont load in right order. maybe saving in XXXXXXXX_d4150... format is best


	//// 0.5. calculate normals 
	//std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> clouds_normals;
	//estimate_normals(clouds, clouds_normals);

	// 1. mls
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> clouds_smoothed;

	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
	pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;
	//mls.setComputeNormals(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);  // should be same size that was used to estimate normals, if normals were precalculated
	for (int i = 0; i < 2; i++) {  // clouds.size()
		printf("mls %i/%i\n", i+1, clouds.size());

		mls.setInputCloud(clouds[i]);

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointNormal>());
		mls.process(*cloud_smoothed);

		clouds_smoothed.push_back(cloud_smoothed);

		// debug
		pcl::io::savePLYFile(out_dir + cloud_filenames[i] + "_mls.ply", *cloud_smoothed);
	}
	

	// 2. spsr
	std::vector<pcl::PolygonMesh::Ptr> meshes_spsr;

	for (int i = 0; i < clouds_smoothed.size(); i++) {
		printf("spsr %i/%i\n", i + 1, clouds_smoothed.size());

		pcl::PolygonMesh::Ptr mesh_spsr(boost::make_shared<pcl::PolygonMesh>());
		volcap::surface::compute_mesh<pcl::PointNormal>(clouds_smoothed[i], *mesh_spsr);

		meshes_spsr.push_back(mesh_spsr);

		// debug
		pcl::io::savePLYFile(out_dir + cloud_filenames[i] + "_spsr.ply", *mesh_spsr);
	}
	
	// 2.5 decimation
	float percentage_to_remove = 0.8;
	std::vector<pcl::PolygonMesh::Ptr> meshes_decimated;

	for (int i = 0; i < meshes_spsr.size(); i++) {
		printf("decimation %i/%i\n", i + 1, meshes_spsr.size());

		pcl::PolygonMesh::Ptr mesh_decimated(new pcl::PolygonMesh());
		volcap::surface::decimate_mesh(meshes_spsr[i], *mesh_decimated, percentage_to_remove);

		meshes_decimated.push_back(mesh_decimated);

		// debug
		pcl::io::savePLYFile(out_dir + cloud_filenames[i] + "_decimated.ply", *mesh_decimated);
	}


	// 3. uv_map
	float maxStretch = 0.6;
	std::vector<pcl::TextureMesh::Ptr> meshes_uv;

	for (int i = 0; i < meshes_decimated.size(); i++) {
		printf("uv_map %i/%i\n", i + 1, meshes_decimated.size());

		pcl::TextureMesh::Ptr mesh_uv(new pcl::TextureMesh());
		volcap::texture::generateUVMapping(*meshes_decimated[i], *mesh_uv, maxStretch);

		meshes_uv.push_back(mesh_uv);

		// debug
		pcl::io::saveOBJFile(out_dir + cloud_filenames[i] + "_uv-map.obj", *mesh_uv);
	}

	// 4. texturing
	//std::vector<pcl::TextureMesh::Ptr> meshes_tex;
	volcap::texture::Texturing t;

	//==> prepare image files
	printf("preparing img files for texturing\n");

	//std::map<std::string, int> cam_idxs;
	//std::array<std::vector<std::string>, meshes_uv.size()> img_lists;
	std::vector<std::vector<std::string>> img_lists(meshes_uv.size());
	boost::filesystem::directory_iterator it{ images_dir_path };
	while (it != boost::filesystem::directory_iterator{}) {
		// example img file: 0_d4150_color_30466.png

		// Get input / output paths
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path path = entry.path();
		std::string file_stem = path.stem().string();

		// split at underscores
		std::vector<std::string> tokens;
		std::string token;
		std::istringstream tokenStream(file_stem);
		while (std::getline(tokenStream, token, '_'))
		{
			tokens.push_back(token);
		}

		int mesh_idx = std::stoi(tokens[0]);
		std::string cam = tokens[1];

		if (mesh_idx < meshes_uv.size())
			img_lists[mesh_idx].push_back(path.string());
		else
			printf("volcap warn: cloud for mesh %u potentially missing\n", mesh_idx);
	}

	//==> compute texturing
	for (int i = 0; i < meshes_uv.size(); i++) {
		printf("texturing %i/%i\n", i + 1, meshes_uv.size());
		//pcl::TextureMesh::Ptr mesh_tex(new pcl::TextureMesh());
		pcl::TextureMesh::Ptr mesh_tex = meshes_uv[i];

		//==> segment using custom func
		std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> tex_coords;
		std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> img_coords;
		std::vector<std::vector<int>> tri_verts;
		std::vector<std::vector<float>> cam_weights;
		t.segmentUVMeshByCamera(*meshes_uv[i], cams, tex_coords, img_coords, tri_verts, cam_weights);

		//==> generate texture map using UVAtlas' uv-map, greedy custom segmentation
		std::string texture_file = cloud_filenames[i] + ".bmp";
		std::string texture_file_full = out_dir + texture_file;
		t.generateUVTextureFromImages(texture_file_full, tex_coords, img_coords, img_lists[i], tri_verts, cam_weights);

		//==> update TextureMesh material to use new texture file
		//mesh.tex_materials[0].tex_file = texture_file_full;  // saving full path in texture material
		mesh_tex->tex_materials[0].tex_file = texture_file;  // saving relative path in texture material (preferred if it works)
		
		//pcl::TexMaterial tex_mat;
		//tex_mat.tex_file = texture_file;
		//mesh_tex->tex_materials.push_back(tex_mat);

		//meshes_tex.push_back(mesh_tex);

		// debug
		pcl::io::saveOBJFile(out_dir + cloud_filenames[i] + "_textured.obj", *mesh_tex);
	}

	// 5. output to file (fbx / usd / obj)
	
}

int main(int argc, char *argv[]) {

	try {
		std::string input_dir;
		std::string output_dir;
		std::string calib_dir;
		std::string output_format;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help", "produce help message")
			("input-dir", po::value<std::string>(&input_dir), "input directory")
			("output-dir", po::value<std::string>(&output_dir), "output directory")
			("calib-dir", po::value<std::string>(&calib_dir), "calibration directory")
			("output-format", po::value<std::string>(&output_format)->default_value("obj"), "output file format ['fbx', 'usda' ,'obj']. default 'obj'")
		;

		po::positional_options_description p;
		p.add("input-dir", 1);
		p.add("output-dir", 1);
		p.add("calib-dir", 1);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);

		// set `output_dir`, `input_dir` if not specified
		boost::filesystem::path input_path(input_dir);
		if (output_dir == "") {
			boost::filesystem::path output_path = input_path / "mesh";
			output_dir = output_path.string();
		}

		if (calib_dir == "") {
			boost::filesystem::path calib_path = input_path / "calibration";
			calib_dir = calib_path.string();
		}

		if (vm.count("help")) {
			std::cout << desc << "\n";
			return 0;
		}

		if (output_format != "obj" && output_format != "usda" && output_format != "fbx") {
			std::cerr << "Output file format not one of ['fbx', 'usda' ,'obj']!";
			return 1;
		}

		std::vector<volcap::io::Camera*> cams;
		volcap::io::loadCameraParams_VCL(calib_dir, cams);

		output_dir += "/";
		reconstruction_pipeline(input_dir, output_dir, cams);
	}
	catch (std::exception& e) {
		std::cerr << "error: " << e.what() << "\n";
		return 1;
	}
	catch (...) {
		std::cerr << "Exception of unknown type!\n";
	}

	return 0;

}
