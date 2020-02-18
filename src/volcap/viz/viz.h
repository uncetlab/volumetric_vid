#include <string>

//! the volcap namespace
namespace volcap {

	//! the viz namespace
	namespace viz {

		/** 
		 * @brief Visualize a single mesh seq
		 * (method takes type as an arg instead of being templated bc we are unable to easily cast templates in c++,
		 *  and the PCL methods themselves are not templated, requiring different function calls dependent on type)
		 *
		 * inputs:
		 *   - // template type T: PolygonMeshPtr or TextureMeshPtr
		 *   - dir_name:
		 *   - type: "PolygonMesh" or "TextureMesh" are valid
		 */
		void viz_mesh_seq(const std::string dir_name, const std::string type = "PolygonMesh");

		/**
		 * @brief Visualize two sequences side by side
		 *
		 * inputs:
		 *  - dir_name1:
		 *  - dir_name2:
		 *  - title_1: caption for left visualization
		 *  - title_2: caption for right visualization
		 *  - type1: "mesh" or "cloud" are only valid types
		 *  - type2: "mesh" or "cloud" are only valid types
		 */
		void viz_seq_dual(const std::string dir_name1, const std::string dir_name2, const std::string title_1, const std::string title_2, const std::string type1 = "mesh", const std::string type2 = "mesh");

		/**
		 * @brief Visualize two static point clouds side by side
		 */
		void viz_pc_dual(const std::string path1, const std::string path2, const std::string title_1, const std::string title_2);

		void viz_single_obj(const std::string obj_path);

		//void viz_mesh_seq_triple();

	}
}
