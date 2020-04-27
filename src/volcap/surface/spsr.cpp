#include <volcap/surface/spsr.h>
#include <pcl/point_types.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>

void volcap::surface::decimate_mesh(const pcl::PolygonMesh::Ptr &input_mesh, pcl::PolygonMesh &output_mesh, float p) {
	pcl::MeshQuadricDecimationVTK decimator;
	decimator.setTargetReductionFactor(p);
	decimator.setInputMesh(input_mesh);
	decimator.process(output_mesh);
}
