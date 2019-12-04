#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include "SPSR.h"

void compute_mesh(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &input, pcl::PolygonMesh &output, int depth, int solver_divide, int iso_divide, float point_weight) {
	pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
	poisson.setDepth(depth);
	poisson.setSolverDivide(solver_divide);
	poisson.setIsoDivide(iso_divide);
	poisson.setPointWeight(point_weight);
	poisson.setInputCloud(input);
	poisson.reconstruct(output);
}

void decimate_mesh(const pcl::PolygonMesh::Ptr &input_mesh, float p, pcl::PolygonMesh &output_mesh) {
	pcl::MeshQuadricDecimationVTK decimator;
	decimator.setTargetReductionFactor(p);
	decimator.setInputMesh(input_mesh);
	decimator.process(output_mesh);
}