#include <volcap/texture/uv_map.h>
#include <UVAtlas.h>
#include <DirectXMesh.h>
#include <memory>
#include <WaveFrontReader.h>
#include <stdio.h>
//#include <DirectXMath.h>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

HRESULT __cdecl print_percentage(float percentComplete) {
	printf("progress: %f\n", percentComplete);
	return 0;
}

void volcap::texture::generateUVMapping(pcl::PolygonMesh &pmesh, pcl::TextureMesh &tmesh, float maxStretch) {

	// Prepare input for UVAtlas Create
	size_t nFaces = pmesh.polygons.size();
	size_t nVerts = pmesh.cloud.height * pmesh.cloud.width;

	// build vertices XMFLOAT3[] arr
	//pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr vertices(new pcl::PointCloud<pcl::PointNormal>);
	pcl::fromPCLPointCloud2(pmesh.cloud, *vertices);

	std::unique_ptr<DirectX::XMFLOAT3[]> pos(new DirectX::XMFLOAT3[nVerts]);
	for (int i = 0; i < vertices->size(); i++)
	{
		pcl::PointNormal &v = vertices->points[i];

		pos[i].x = v.data[0];
		pos[i].y = v.data[1];
		pos[i].z = v.data[2];
	}

	// build indicies
	std::unique_ptr<uint16_t[]> indicies(new uint16_t[nFaces * 3]);
	int indicies_idx = 0;
	for (int i = 0; i < pmesh.polygons.size(); i++) {
		indicies[indicies_idx] = pmesh.polygons[i].vertices[0];
		indicies[indicies_idx + 1] = pmesh.polygons[i].vertices[1];
		indicies[indicies_idx + 2] = pmesh.polygons[i].vertices[2];

		indicies_idx += 3;
	}

	std::unique_ptr<uint32_t[]> adj(new uint32_t[nFaces * 3]);
	if (FAILED(DirectX::GenerateAdjacencyAndPointReps(indicies.get(), nFaces,
		pos.get(), nVerts, 0.f, nullptr, adj.get()))) {
		printf("ERROR: GenerateAdjacencyAndPointReps");
		return;
	}

	std::vector<DirectX::UVAtlasVertex> vb;
	std::vector<uint8_t> ib;
	std::vector<uint32_t> remap;

	std::function<HRESULT __cdecl(float percentComplete)> progress_callback = print_percentage;

	int texture_width = 512;
	int texture_height = 512;
	HRESULT hr = DirectX::UVAtlasCreate(
		pos.get(),				// vertex positions
		nVerts,					// # of vertices
		indicies.get(), // indicies of verticies which make up a face (every 3 is a tri)
		DXGI_FORMAT_R16_UINT,	// index format
		nFaces,					// # of faces
		0, maxStretch,					// max # of charts, max stretch param
		texture_width, texture_height,  // width and height the atlas will be used on
		1.f,					// the minimum distance, in texels between two charts on the atlas
		adj.get(),				//
		nullptr, nullptr,
		nullptr, //progress_callback,		// possible callback function
		DirectX::UVATLAS_DEFAULT_CALLBACK_FREQUENCY,
		DirectX::UVATLAS_DEFAULT,
		vb,  // [output][vector<UVAtlasVertex>] vertex buffer: stores tuple (old 3d pos?, corresponding 2d uv position)
		ib,  // [output][vector<uint8_t>] index buffer: stores face information by storing the indicies of data in `vertex buffer`. every 3 is a tri?
		nullptr,
		&remap  // [output] vertex remap array: element ids the orig vertex that each final vertex came from (necessary only if a vertex was split)
	);
	if (FAILED(hr)) {
		printf("ERROR: UVAtlasCreate");
		return;
	}

	size_t nTotalVerts = vb.size();
	std::unique_ptr<WaveFrontReader<uint16_t>::Vertex[]> newVB(
		new WaveFrontReader<uint16_t>::Vertex[nTotalVerts]);

	// converts vb [vector<UVAtlasVertex>[nTotalVerts]] to newVB [Vertex[nTotalVerts]]
	// UVAtlas stores: 3D position, 2D uv position
	// Vertex stores : 3D position, 2D uv position, 3D normal
	// so this is basically just needed to convert vb to the Vertex format, which contains normal information 
	hr = DirectX::UVAtlasApplyRemap(
		pos.get(),	// original vertex buffer
		sizeof(DirectX::XMFLOAT3),  // size of each element in arr above
		nVerts,						// # of 3D vertices
		nTotalVerts,				// # of uv vertices
		&remap.front(),				// vertex remap array
		newVB.get()					// new vertex buffer
	);
	if (FAILED(hr)) {
		printf("ERROR: UVAtlasApplyRemap");
		return;
	}

	// Merge the UV data into the final VB
	for (size_t j = 0; j < nTotalVerts; ++j) {
		newVB[j].textureCoordinate = vb[j].uv;
		//(*newVB)[j].textureCoordinate = vb[j].uv;
	}

	auto newIB = reinterpret_cast<const uint16_t*>(&ib.front());
	// now we have our newVB / newIB! now we render with a fake texture to get our uv map

	//for (int i = 0; i < ib.size() / 2; i++) {
	//	printf("ib[%i]: %i | newIB[%i]: %i\n", i, ib[i], i, newIB[i]);
	//}

	//*** build TextureMesh! ***//

	// create PCL vertex buffer equivalent (contains duplicates currently)
	pcl::PointCloud<pcl::PointNormal> xyz;

	//for (int i = 0; i < nVerts; i++) {  // copy newVB to tmesh.cloud
	//	WaveFrontReader<uint16_t>::Vertex wf_v = newVB[i];
	//	pcl::PointNormal pcl_v;

	//	pcl_v.x = wf_v.position.x;
	//	pcl_v.y = wf_v.position.y;
	//	pcl_v.z = wf_v.position.z;
	//	pcl_v.normal_x = wf_v.normal.x;
	//	pcl_v.normal_y = wf_v.normal.y;
	//	pcl_v.normal_z = wf_v.normal.z;

	//	xyz.push_back(pcl_v);
	//}
	//for (int i = 0; i < nTotalVerts; i++) {  // copy newVB to tmesh.cloud
	//	WaveFrontReader<uint16_t>::Vertex wf_v = newVB[i];
	//	pcl::PointNormal pcl_v;

	//	pcl_v.x = wf_v.position.x;
	//	pcl_v.y = wf_v.position.y;
	//	pcl_v.z = wf_v.position.z;
	//	pcl_v.normal_x = wf_v.normal.x;
	//	pcl_v.normal_y = wf_v.normal.y;
	//	pcl_v.normal_z = wf_v.normal.z;

	//	xyz.push_back(pcl_v);
	//}
	//pcl::toPCLPointCloud2(xyz, tmesh.cloud);
	pcl::toPCLPointCloud2(*vertices, tmesh.cloud);
	//tmesh.cloud = pmesh.cloud;

	// create PCL index buffer equivalent
	std::vector<pcl::Vertices> mesh_poly;
	std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >
		mesh_tex;  // indicies correspond to elements in tmesh.cloud? // should be same shape as index buffer?

	for (int i = 0; i < ib.size() / 2; i += 3) {  // build each face  // ib.size() / 2
		if (i + 2 >= ib.size() / 2) {
			printf("ERROR");
			return;
		}

		// compute idxs
		int idx_1 = newIB[i];
		int idx_2 = newIB[i + 1];
		int idx_3 = newIB[i + 2];

		// apply remap
		int remapped_idx_1 = remap[idx_1];
		int remapped_idx_2 = remap[idx_2];
		int remapped_idx_3 = remap[idx_3];

		pcl::Vertices v;  // indicies correspond to elements in tmesh.cloud?
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
	tmesh.tex_polygons.push_back(mesh_poly);
	tmesh.tex_coordinates.push_back(mesh_tex);

	////==> generate UV gradient map using UVAtlas' generated uv coords
	//std::string tex_filename = file_name + ".bmp";
	//std::string tex_path = out_dir + tex_filename;
	//generateGradientTexture(tex_path, tmesh.tex_coordinates[0]);

	//==> create PCL TexMaterial
	pcl::TexMaterial mesh_material;
	//std::string tex_filename_png = file_name + ".png";
	//mesh_material.tex_file = tex_filename;  // should be in same folder as output .obj
	//mesh_material.tex_file = tex_filename_png;  // should be in same folder as output .obj
	mesh_material.tex_name = "material_0";
	tmesh.tex_materials.push_back(mesh_material);
}
