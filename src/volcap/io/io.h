#include <vector>
#include <string>
#include <pcl/TextureMesh.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>

//! the volcap namespace
namespace volcap {

	//! the io namespace
	namespace io {

		/**
		 * @brief loads all PolygonMeshes (saved as .ply files) from a dir (assumes every file is a .ply file)
		 *
		 * @param[in] dir_name
		 * @param[out] meshes
		 * @param[out] mesh_paths	mesh full paths
		 */
		void load_meshes_from_dir(
			const std::string dir_name,
			std::vector<pcl::PolygonMeshPtr> &meshes,
			std::vector<std::string> &mesh_paths
		);

		/**
		 * @brief loads all TextureMeshes (saved as .obj files) from a dir (assumes every file is a .obj file)
		 *
		 * @remark .obj file must only contain 1 material to work properly
		 *
		 * @param[in] dir_name
		 * @param[out] meshes
		 * @param[out] mesh_filenames	mesh file names (without extensions)
		 */
		void load_meshes_from_dir(
			const std::string dir_name,
			std::vector<pcl::TextureMeshPtr> &meshes,
			std::vector<std::string> &mesh_filenames
		);

		/**
		 * @brief loads all clouds in `dir_name` and stores them in `clouds`. assumes each file is a .ply point cloud
		 */
		void load_clouds_from_dir(const std::string dir_name, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds, std::vector<std::string> &ids);

		///* loads all PolygonMeshes / TextureMeshes from a dir (assumes every file is a .obj / .ply file)
		// * assumes PolygonMeshes are stored in .ply files
		// * assumes TextureMeshes are stored in .obj files
		// *
		// * input:
		// *   - template type T: PolygonMeshPtr or TextureMeshPtr
		// */
		//template <class T>
		//void load_meshes_from_dir(const std::string dir_name, std::vector<T> &meshes, std::vector<std::string> &mesh_ids);
	}
}



