#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>

void example_tri(pcl::TextureMesh &mesh) {
	// create PCL vertex buffer equivalent
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

	// create PCL index buffer equivalent
	std::vector<pcl::Vertices> mesh_poly;

	pcl::Vertices v;
	v.vertices.push_back(0);
	v.vertices.push_back(1);
	v.vertices.push_back(2);
	mesh_poly.push_back(v);

	// create PCL UV mapping
	std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >
		mesh_tex;

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

	// create PCL TexMaterial
	pcl::TexMaterial mesh_material;

	mesh_material.tex_file = "C:\\Users\\maxhu\\Desktop\\uvatlas_example\\uv_gradient.jpg";
	mesh_material.tex_name = "material_0";

	mesh.tex_polygons.push_back(mesh_poly);
	mesh.tex_coordinates.push_back(mesh_tex);
	mesh.tex_materials.push_back(mesh_material);

	//pcl::io::saveOBJFile("C:\\Users\\maxhu\\Desktop\\uvatlas_example\\test.obj", mesh);
}

//void example_cube(pcl::TextureMesh &mesh) {
void example_cube(pcl::PolygonMesh &pmesh) {
	//pcl::PolygonMesh mesh;
	// create PCL vertex buffer equivalent
	pcl::PointCloud<pcl::PointNormal> xyz;

	for (int x = 0; x < 2; x++) {
		for (int y = 0; y < 2; y++) {
			for (int z = 0; z < 2; z++) {
				pcl::PointNormal pt;
				pt.x = x;
				pt.y = y;
				pt.z = z;
				xyz.push_back(pt);
			}
		}
	}
	pcl::toPCLPointCloud2(xyz, pmesh.cloud);

	// create PCL index buffer equivalent
	std::vector<pcl::Vertices> mesh_poly;

	pcl::Vertices f1;
	f1.vertices.push_back(0);
	f1.vertices.push_back(6);
	f1.vertices.push_back(4);

	pcl::Vertices f2;
	f2.vertices.push_back(0);
	f2.vertices.push_back(2);
	f2.vertices.push_back(6);

	pcl::Vertices f3;
	f3.vertices.push_back(0);
	f3.vertices.push_back(3);
	f3.vertices.push_back(2);

	pcl::Vertices f4;
	f4.vertices.push_back(0);
	f4.vertices.push_back(1);
	f4.vertices.push_back(3);

	pcl::Vertices f5;
	f5.vertices.push_back(2);
	f5.vertices.push_back(7);
	f5.vertices.push_back(6);

	pcl::Vertices f6;
	f6.vertices.push_back(2);
	f6.vertices.push_back(3);
	f6.vertices.push_back(7);

	pcl::Vertices f7;
	f7.vertices.push_back(4);
	f7.vertices.push_back(6);
	f7.vertices.push_back(7);

	pcl::Vertices f8;
	f8.vertices.push_back(4);
	f8.vertices.push_back(7);
	f8.vertices.push_back(5);

	pcl::Vertices f9;
	f9.vertices.push_back(0);
	f9.vertices.push_back(4);
	f9.vertices.push_back(5);

	pcl::Vertices f10;
	f10.vertices.push_back(0);
	f10.vertices.push_back(5);
	f10.vertices.push_back(1);

	pcl::Vertices f11;
	f11.vertices.push_back(1);
	f11.vertices.push_back(5);
	f11.vertices.push_back(7);

	pcl::Vertices f12;
	f12.vertices.push_back(1);
	f12.vertices.push_back(7);
	f12.vertices.push_back(3);

	//mesh_poly.push_back(f1);
	//mesh_poly.push_back(f2);
	//mesh_poly.push_back(f3);
	//mesh_poly.push_back(f4);
	//mesh_poly.push_back(f5);
	//mesh_poly.push_back(f6);
	//mesh_poly.push_back(f7);
	//mesh_poly.push_back(f8);
	//mesh_poly.push_back(f9);
	//mesh_poly.push_back(f10);
	//mesh_poly.push_back(f11);
	//mesh_poly.push_back(f12);

	pmesh.polygons.push_back(f1);
	pmesh.polygons.push_back(f2);
	pmesh.polygons.push_back(f3);
	pmesh.polygons.push_back(f4);
	pmesh.polygons.push_back(f5);
	pmesh.polygons.push_back(f6);
	pmesh.polygons.push_back(f7);
	pmesh.polygons.push_back(f8);
	pmesh.polygons.push_back(f9);
	pmesh.polygons.push_back(f10);
	pmesh.polygons.push_back(f11);
	pmesh.polygons.push_back(f12);

	//// create PCL UV mapping
	//std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >
	//	mesh_tex;

	//Eigen::Vector2f tex1;
	//Eigen::Vector2f tex2;
	//Eigen::Vector2f tex3;
	//tex1(0) = -1.0;
	//tex1(1) = 0.0;
	//tex2(0) = -1.0;
	//tex2(1) = 1.0;
	//tex3(0) = 2.0;
	//tex3(1) = 0.0;

	//mesh_tex.push_back(tex1);
	//mesh_tex.push_back(tex2);
	//mesh_tex.push_back(tex3);

	//// create PCL TexMaterial
	//pcl::TexMaterial mesh_material;

	//mesh_material.tex_file = "C:\\Users\\maxhu\\Desktop\\uvatlas_example\\uv_gradient.jpg";
	//mesh_material.tex_name = "material_0";

	//mesh.tex_polygons.push_back(mesh_poly);
	//mesh.tex_coordinates.push_back(mesh_tex);
	//mesh.tex_materials.push_back(mesh_material);

	//pcl::io::saveOBJFile("C:\\Users\\maxhu\\Desktop\\uvatlas_example\\test_cube.obj", mesh);
}

int main() {
	//pcl::TextureMesh mesh;
	pcl::PolygonMesh pmesh;
	//example_tri(mesh);
	example_cube(pmesh);

	// visualize
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setPosition(0, 0);
	viewer->setSize(1173, 732);
	viewer->addCoordinateSystem(3.0);
	//viewer->addTextureMesh(mesh, "mesh_0");
	viewer->addPolygonMesh(pmesh, "pmesh_0");
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}