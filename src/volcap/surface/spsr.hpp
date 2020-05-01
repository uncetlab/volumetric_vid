#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>

template <typename PointT>
void volcap::surface::compute_mesh(
	typename const pcl::PointCloud<PointT>::Ptr &input,
	pcl::PolygonMesh &output,
	int depth, int solver_divide, int iso_divide, float point_weight
) {
	pcl::Poisson<PointT> poisson;
	poisson.setDepth(depth);
	poisson.setSolverDivide(solver_divide);
	poisson.setIsoDivide(iso_divide);
	poisson.setPointWeight(point_weight);
	poisson.setInputCloud(input);
	poisson.reconstruct(output);

	// copy normal information into output's cloud?

}

template <typename PointT>
void volcap::surface::compute_mesh(
	typename const pcl::PointCloud<PointT>::Ptr &input,
	typename pcl::PointCloud<PointT> &out_points,
	std::vector<pcl::Vertices> &out_polygons,
	int depth, int solver_divide, int iso_divide, float point_weight
) {
	pcl::Poisson<pcl::PointT> poisson;
	poisson.setDepth(depth);
	poisson.setSolverDivide(solver_divide);
	poisson.setIsoDivide(iso_divide);
	poisson.setPointWeight(point_weight);
	poisson.setInputCloud(input);
	poisson.reconstruct(out_points, out_polygons);
}
