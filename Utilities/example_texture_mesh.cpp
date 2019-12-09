#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main() {
	pcl::TextureMesh mesh;
	pcl::PointCloud<pcl::PointNormal> xyz;

	pcl::PointNormal point1;
	pcl::PointNormal point2;
	pcl::PointNormal point3;
	point1.x = 0;
	point2.x = 1;
	point3.x = 0;
	point1.y = 0;
	point2.y = 0;
	point3.y = 1;
	point1.z = 2;
	point2.z = 2;
	point3.z = 2;
	xyz.push_back(point1);
	xyz.push_back(point2);
	xyz.push_back(point3);
	pcl::toPCLPointCloud2(xyz, mesh.cloud);

	std::vector<pcl::Vertices> mesh_poly;
	std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >
		mesh_tex;
	pcl::TexMaterial mesh_material;

	pcl::Vertices v;
	v.vertices.push_back(0);
	v.vertices.push_back(1);
	v.vertices.push_back(2);
	mesh_poly.push_back(v);

	Eigen::Vector2f tex1;
	Eigen::Vector2f tex2;
	Eigen::Vector2f tex3;
	tex1(0) = -1.0;
	tex1(1) = 0.0;
	tex2(0) = -1.0;
	tex2(1) = 1.0;
	tex3(0) = 2.0;
	tex3(1) = 0.0;

	mesh_tex.push_back(tex1);
	mesh_tex.push_back(tex2);
	mesh_tex.push_back(tex3);

	mesh_material.tex_file = "C:\\Users\\maxhu\\Desktop\\uvatlas_example\\uv_gradient.jpg";
	mesh_material.tex_name = "material_0";

	mesh.tex_polygons.push_back(mesh_poly);
	mesh.tex_coordinates.push_back(mesh_tex);
	mesh.tex_materials.push_back(mesh_material);

	pcl::io::saveOBJFile("C:\\Users\\maxhu\\Desktop\\uvatlas_example\\test.obj", mesh);

	// visualize
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setPosition(0, 0);
	viewer->setSize(1173, 732);
	viewer->addCoordinateSystem(3.0);
	viewer->addTextureMesh(mesh, "tri");
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}