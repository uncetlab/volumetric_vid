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
			  * @brief	converts pcl::TextureMesh to usd, exports to .usda file
			  * @remark assumes the TextureMesh has 1 of each tex_polygons, tex_coordinates, tex_materials
			  */
			 void usdFromTextureMesh(
				 pcl::TextureMeshPtr mesh,
				 const std::string mesh_name
			 );

			 /**
			  * @brief	converts sequence of pcl::TextureMesh to usd, exports to **a single** .usda file
			  * @remark assumes the TextureMeshes have 1 of each tex_polygons, tex_coordinates, tex_materials
			  */
			 void usdFromTextureMeshes(
				 std::vector<pcl::TextureMeshPtr> meshes,
				 std::vector<std::string> mesh_names
			 );

			 /** 
			  * @brief	applies an additional rotation to the pxr::UsdGeomMesh stored within this class
			  * @remark	x rotation applies first, y rotation second, z rotation last
			  */
			 void rotateXYZ(float deg_x, float deg_y, float deg_z);

			 //! saves to .usda (at the path specified during object creation)
			 void save();

		private:
			pxr::UsdStageRefPtr stage;
			pxr::UsdGeomMesh usdMesh;

			/**
			 * @brief converts a pcl::TextureMesh to pxr::UsdGeomMesh (storing result in class variable `usdMesh`)
			 *
			 * @param[in] mesh	the mesh to be converted
			 * @param[in] time	optional. the timecode for the mesh to be placed at. used for sequences of >1 mesh.
			 */
			void addMesh(pcl::TextureMesh &mesh, pxr::UsdTimeCode time = pxr::UsdTimeCode::Default());
		};

	}
}
