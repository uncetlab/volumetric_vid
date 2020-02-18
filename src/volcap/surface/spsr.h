#pragma once
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
		 * @brief test brief
		 */
		void compute_mesh(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &input, pcl::PolygonMesh &output,
			int depth = DEFAULT_DEPTH, int solver_divide = DEFAULT_SOLVER_DIVIDE, int iso_divide = DEFAULT_ISO_DIVIDE, float point_weight = DEFAULT_POINT_WEIGHT);

		void compute_mesh(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &input, pcl::PointCloud<pcl::PointXYZRGBNormal> &out_points, std::vector<pcl::Vertices> &out_polygons,
			int depth = DEFAULT_DEPTH, int solver_divide = DEFAULT_SOLVER_DIVIDE, int iso_divide = DEFAULT_ISO_DIVIDE, float point_weight = DEFAULT_POINT_WEIGHT);

		/**
		 *  p: percentage of faces to remove
		 */
		void decimate_mesh(const pcl::PolygonMesh::Ptr &input_mesh, float p, pcl::PolygonMesh &output_mesh);
	}
}
