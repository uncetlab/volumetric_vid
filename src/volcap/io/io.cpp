#include <volcap/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <boost/filesystem.hpp>

void volcap::io::load_meshes_from_dir(
	const std::string dir_name,
	std::vector<pcl::PolygonMeshPtr> &meshes,
	std::vector<std::string> &mesh_ids
) {
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

void volcap::io::load_meshes_from_dir(
	const std::string dir_name,
	std::vector<pcl::TextureMeshPtr> &meshes,
	std::vector<std::string> &mesh_filenames  // std::vector<std::string> &mesh_ids
) {
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

		//edit texture files to be full paths (necessary when creating textures) -- it does this by default? --mh 3.11.20
		//mesh->tex_materials[0].tex_file = dir_name + "/" + mesh->tex_materials[0].tex_file;

		meshes.push_back(mesh);
		//mesh_ids.push_back(mesh_path);

		std::string filename = path.stem().string();  // .filename() includes extension, use .stem() instead
		mesh_filenames.push_back(filename);
	}
}

///* loads all TextureMeshes (saved as .obj files) from a dir (assumes every file is a .obj file)
// * works with .obj files which only have 1 material
// *
// * @todo: load this method from another file instead of pasting here
// */
//void load_meshes_from_dir(const std::string dir_name, std::vector<pcl::TextureMeshPtr> &meshes, std::vector<std::string> &mesh_filenames) {
//	boost::filesystem::path input_dir(dir_name);
//
//	boost::filesystem::directory_iterator it{ input_dir };
//	int i = 0;
//	while (it != boost::filesystem::directory_iterator{}) {
//		// Get input / output paths
//		boost::filesystem::directory_entry entry = *it++;
//		boost::filesystem::path path = entry.path();
//		std::string filename = path.stem().string();  // .filename() includes extension, use .stem() instead
//
//		if (boost::filesystem::extension(path) != ".obj") {
//			printf("skipping file with extension: %s\n", boost::filesystem::extension(path).c_str());
//			continue;
//		}
//		printf("loading TextureMesh %i\n", i++);
//
//		std::string mesh_path = path.string();
//
//		pcl::TextureMeshPtr mesh(boost::make_shared<pcl::TextureMesh>());
//		pcl::io::loadOBJFile(mesh_path, *mesh);		// this is broken for TextureMeshes with multiple materials, 
//													// all texture coordinates get loaded into the first submesh 
//													// (any other submeshes get no texture coordinates)
//
//		//edit texture files to be full paths (necessary when creating textures)
//		mesh->tex_materials[0].tex_file = dir_name + "/" + mesh->tex_materials[0].tex_file;
//
//		meshes.push_back(mesh);
//		mesh_filenames.push_back(filename);
//	}
//}

void volcap::io::load_clouds_from_dir(const std::string dir_name, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds, std::vector<std::string> &ids) {
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
