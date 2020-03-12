#include <pcl/TextureMesh.h>
#include <string>
#include <vector>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/mesh.h>

//! the volcap namespace
namespace volcap {

	//! the io namespace
	namespace io {

		class UsdExporter {
		public:

			 UsdExporter(std::string output_full_path);

			 /**
			 * @brief converts pcl::TextureMesh to usd, exports to .usda file
			 * @remark assumes the TextureMesh has 1 of each tex_polygons, tex_coordinates, tex_materials
			 */
			 //void usdFromTextureMesh(pcl::TextureMesh &mesh, const std::string mesh_name);

			 /**
			 * @brief converts sequence of pcl::TextureMesh to usd, exports to .usda file
			 * @remark assumes the TextureMesh has 1 of each tex_polygons, tex_coordinates, tex_materials
			 */
			 void usdFromTextureMeshes(
				 std::vector<pcl::TextureMeshPtr> meshes,
				 std::vector<std::string> mesh_names
			 );

			 void save();

		private:
			pxr::UsdStageRefPtr stage;
			pxr::UsdGeomMesh usdMesh;

			void addMesh(pcl::TextureMesh &mesh, int time = 0);
		};

	}
}
