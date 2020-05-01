#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>


//! the volcap namespace
namespace volcap {

	//! the texture namespace
	namespace texture {

		/**
		 * @brief uses UVAtlas to create uv map for a given mesh
		 *
		 * @param[in] pmesh		a poly mesh with a valid `pmesh.cloud` and `pmesh.polygons`
		 * @param[out] tmesh	uv-mapping gets placed in `tmesh.tex_coordinates`, other fields are copied from `pmesh`
		 */
		void generateUVMapping(
			pcl::PolygonMesh &pmesh,
			pcl::TextureMesh &tmesh,
			float maxStretch=0.f
		);
	}
}