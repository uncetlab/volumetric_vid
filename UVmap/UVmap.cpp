/*
 * Shows example of Microsoft UVAtlas being used to create a UV mapping for a simple .obj cube
 * Outputs to .obj
 *
 */
#include <UVAtlas.h>
#include <DirectXMesh.h>
#include <memory>
#include <WaveFrontReader.h>
#include <stdio.h>
//#include <DirectXMath.h>
#include <pcl/TextureMesh.h>
#include <pcl/common/projection_matrix.h>
//#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/obj_io.h>


int main(int argc, char** argv)
{
	auto mesh = std::make_unique<WaveFrontReader<uint16_t>>();

	if (FAILED(mesh->Load(L"C:\\Users\\maxhu\\Desktop\\uvatlas_example\\cube.obj"))) {
		printf("Error loading .obj file");
		return 1;
	}

	// Prepare input for UVAtlas Create
	size_t nFaces = mesh->indices.size() / 3;
	size_t nVerts = mesh->vertices.size();

	std::unique_ptr<DirectX::XMFLOAT3[]> pos(new DirectX::XMFLOAT3[nVerts]);
	for (size_t j = 0; j < nVerts; ++j)
		pos[j] = mesh->vertices[j].position;

	std::unique_ptr<uint32_t[]> adj(new uint32_t[mesh->indices.size()]);
	if (FAILED(DirectX::GenerateAdjacencyAndPointReps(&mesh->indices.front(), nFaces,
		pos.get(), nVerts, 0.f, nullptr, adj.get()))) {
		printf("ERROR: GenerateAdjacencyAndPointReps");
		return 1;
	}

	std::vector<DirectX::UVAtlasVertex> vb;
	std::vector<uint8_t> ib;
	std::vector<uint32_t> remap;

	int texture_width = 512;
	int texture_height = 512;
	HRESULT hr = DirectX::UVAtlasCreate(
		pos.get(),				// vertex positions
		nVerts,					// # of vertices
		&mesh->indices.front(), // indicies of verticies which make up a face (every 3 is a tri)
		DXGI_FORMAT_R16_UINT,	// index format
		nFaces,					// # of faces
		0, 0.f,					// max # of charts, max stretch param
		texture_width, texture_height,  // width and height the atlas will be used on
		1.f,					// the minimum distance, in texels between two charts on the atlas
		adj.get(),				// ?? necessary, can't be nullptr
		nullptr, nullptr,
		nullptr,				// possible callback function
		DirectX::UVATLAS_DEFAULT_CALLBACK_FREQUENCY,
		DirectX::UVATLAS_DEFAULT, 
		vb,  // [output][vector<UVAtlasVertex>] vertex buffer: stores tuple (old 3d pos?, corresponding 2d uv position)
		ib,  // [output][vector<uint8_t>] index buffer: stores face information by storing the indicies of data in `vertex buffer`. every 3 is a tri?
		nullptr,
		&remap  // [output] vertex remap array: element ids the orig vertex that each final vertex came from (necessary only if a vertex was split)
	);
	if (FAILED(hr)) {
		printf("ERROR: UVAtlasCreate");
		return 2;
	}

	size_t nTotalVerts = vb.size();
	std::unique_ptr<WaveFrontReader<uint16_t>::Vertex[]> newVB(
		new WaveFrontReader<uint16_t>::Vertex[nTotalVerts]);

	// converts vb [vector<UVAtlasVertex>[nTotalVerts]] to newVB [Vertex[nTotalVerts]]
	// UVAtlas stores: 3D position, 2D uv position
	// Vertex stores : 3D position, 2D uv position, 3D normal
	// so this is basically just needed to convert vb to the Vertex format, which contains normal information 
	hr = DirectX::UVAtlasApplyRemap(
		&mesh->vertices.front(),	// original vertex buffer
		sizeof(WaveFrontReader<uint16_t>::Vertex),  // size of each element
		nVerts,						// # of 3D vertices
		nTotalVerts,				// # of uv vertices
		&remap.front(),				// vertex remap array
		newVB.get()					// new vertex buffer
	);
	if (FAILED(hr)) {
		printf("ERROR: UVAtlasApplyRemap");
		return 1;
	}

	// Merge the UV data into the final VB
	for (size_t j = 0; j < nTotalVerts; ++j) {
		newVB[j].textureCoordinate = vb[j].uv;
	}
		
	auto newIB = reinterpret_cast<const uint16_t*>(&ib.front());
	// now we have our newVB / newIB! now we render with a fake texture to get our uv map

	for (int i = 0; i < ib.size(); i++) {
		printf("ib[%i]: %i | newIB[%i]: %i\n", i, ib[i], i, newIB[i]);
	}

	//*** build TextureMesh! ***//
	pcl::TextureMesh texture_mesh;

	// create PCL vertex buffer equivalent (contains duplicates currently)
	pcl::PointCloud<pcl::PointNormal> xyz;
	
	for (int i = 0; i < nVerts; i++) {  // copy newVB to texture_mesh.cloud
		WaveFrontReader<uint16_t>::Vertex wf_v = mesh->vertices[i];
		pcl::PointNormal pcl_v;

		pcl_v.x = wf_v.position.x;
		pcl_v.y = wf_v.position.y;
		pcl_v.z = wf_v.position.z;
		pcl_v.normal_x = wf_v.normal.x;
		pcl_v.normal_y = wf_v.normal.y;
		pcl_v.normal_z = wf_v.normal.z;

		xyz.push_back(pcl_v);
	}
	pcl::toPCLPointCloud2(xyz, texture_mesh.cloud);

	// create PCL index buffer equivalent
	std::vector<pcl::Vertices> mesh_poly;
	std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >
		mesh_tex;  // indicies correspond to elements in texture_mesh.cloud? // should be same shape as index buffer?

	for (int i = 0; i < ib.size() / 2; i+=3) {  // build each face  // ib.size() / 2
		if (i + 2 >= ib.size() / 2) {
			printf("ERROR");
			return 1;
		}

		// compute idxs
		int idx_1 = newIB[i];
		int idx_2 = newIB[i + 1];
		int idx_3 = newIB[i + 2];

		// apply remap
		int remapped_idx_1 = remap[idx_1];
		int remapped_idx_2 = remap[idx_2];
		int remapped_idx_3 = remap[idx_3];

		pcl::Vertices v;  // indicies correspond to elements in texture_mesh.cloud?
		v.vertices.push_back(remapped_idx_1);
		v.vertices.push_back(remapped_idx_2);
		v.vertices.push_back(remapped_idx_3);
		mesh_poly.push_back(v);

		// create corresponding uv coords
		
		Eigen::Vector2f tex1;
		Eigen::Vector2f tex2;
		Eigen::Vector2f tex3;
		tex1(0) = newVB[idx_1].textureCoordinate.x;
		tex1(1) = newVB[idx_1].textureCoordinate.y;
		tex2(0) = newVB[idx_2].textureCoordinate.x;
		tex2(1) = newVB[idx_2].textureCoordinate.y;
		tex3(0) = newVB[idx_3].textureCoordinate.x;
		tex3(1) = newVB[idx_3].textureCoordinate.y;

		mesh_tex.push_back(tex1);
		mesh_tex.push_back(tex2);
		mesh_tex.push_back(tex3);

	}
	texture_mesh.tex_polygons.push_back(mesh_poly);
	texture_mesh.tex_coordinates.push_back(mesh_tex);

	// create PCL TexMaterial
	pcl::TexMaterial mesh_material;

	//mesh_material.tex_file = "C:\\Users\\maxhu\\Desktop\\uvatlas_example\\uv_gradient.jpg";
	mesh_material.tex_file = "uv_gradient.jpg";  // should be in same folder as output .obj
	mesh_material.tex_name = "material_0";

	texture_mesh.tex_materials.push_back(mesh_material);

	// MUST be declared with forward slashes to work correctly
	pcl::io::saveOBJFile("C:/Users/maxhu/Desktop/uvatlas_example/test_cube_remapped.obj", texture_mesh);

	//// visualize
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->setPosition(0, 0);
	//viewer->setSize(1173, 732);
	//viewer->addCoordinateSystem(3.0);
	//viewer->addTextureMesh(texture_mesh, "cube");
	//while (!viewer->wasStopped()) {
	//	viewer->spinOnce(100);
	//	//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
	
}
