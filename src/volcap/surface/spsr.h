#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>


//! the volcap namespace
namespace volcap {

	//! the surface namespace
	namespace surface {
		const int DEFAULT_DEPTH = 8;
		const int DEFAULT_SOLVER_DIVIDE = 8;
		const int DEFAULT_ISO_DIVIDE = 8;
		const float DEFAULT_POINT_WEIGHT = 4.0f;

		/**
		 * @brief calls PCL's SPSR implementation
		 *
		 * @param[in] input
		 * @param[out] output
		 * @param[in] depth
		 * @param[in] solver_divide
		 * @param[in] iso_divide
		 * @param[in] point_weight
		 */
		template <typename PointT>
		void compute_mesh(
			typename const pcl::PointCloud<PointT>::Ptr &input,
			pcl::PolygonMesh &output,
			int depth = DEFAULT_DEPTH,
			int solver_divide = DEFAULT_SOLVER_DIVIDE,
			int iso_divide = DEFAULT_ISO_DIVIDE,
			float point_weight = DEFAULT_POINT_WEIGHT
		);

		template <typename PointT>
		void compute_mesh(
			typename const pcl::PointCloud<PointT>::Ptr &input,
			typename pcl::PointCloud<PointT> &out_points,
			std::vector<pcl::Vertices> &out_polygons,
			int depth = DEFAULT_DEPTH,
			int solver_divide = DEFAULT_SOLVER_DIVIDE,
			int iso_divide = DEFAULT_ISO_DIVIDE,
			float point_weight = DEFAULT_POINT_WEIGHT
		);

		/**
		 * @brief calls PCL's mesh decimator
		 *
		 * @param[in] input_mesh	
		 * @param[out] output_mesh
		 * @param[in] p				percentage of faces to remove
		 */
		void decimate_mesh(
			const pcl::PolygonMesh::Ptr &input_mesh,
			pcl::PolygonMesh &output_mesh,
			float p
		);
	}
}

#include <volcap/surface/spsr.hpp>
