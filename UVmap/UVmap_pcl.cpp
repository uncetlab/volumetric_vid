/*
 * Shows example of Microsoft UVAtlas being used to create a UV mapping for a .ply mesh obtained after SPSR reconstruction
 *
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
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include "texturing.h"

HRESULT __cdecl print_percentage(float percentComplete) {
	printf("progress: %f\n", percentComplete);
	return 0;
}

/* loads all PolygonMeshes (saved as .ply files) from a dir (assumes every file is a .ply file)
 * TODO: import this func instead of pasting code here
 */
void load_meshes_from_dir(const std::string dir_name, std::vector<pcl::PolygonMeshPtr> &meshes, std::vector<std::string> &mesh_ids) {
	boost::filesystem::path input_dir(dir_name);

	boost::filesystem::directory_iterator it{ input_dir };
	int i = 0;
	while (it != boost::filesystem::directory_iterator{}) {

		// Get input / output paths
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path path = entry.path();
		//std::string mesh_id = in_p.filename().string();
		std::string mesh_path = path.string();

		pcl::PolygonMeshPtr mesh(boost::make_shared<pcl::PolygonMesh>());
		
		if (boost::filesystem::extension(path) == ".ply") {
			pcl::io::loadPLYFile(mesh_path, *mesh);
		}
		//else if (boost::filesystem::extension(path) == ".obj") {
		//	pcl::io::loadPolygonFileOBJ(mesh_path, *mesh);
		//}
		else {
			printf("skipping file with extension: %s\n", boost::filesystem::extension(path).c_str());
			continue;
		}
		printf("loading PolygonMesh %i\n", i++);


		meshes.push_back(mesh);
		mesh_ids.push_back(mesh_path);
	}
}

int main(int argc, char** argv)
{
	//// generating texture based on coords loaded from .obj (so we dont have to run UVAtlas again)
	//// load TextureMesh from .obj

	////pcl::TextureMeshPtr texture_mesh(boost::make_shared<pcl::TextureMesh>());
	//pcl::TextureMesh texture_mesh;
	//pcl::io::loadOBJFile("C:/Users/maxhu/Desktop/uvatlas_example/test_panoptic.obj", texture_mesh);
	//generateGradientTexture("C:/Users/maxhu/Desktop/uvatlas_example/test_panoptic.bmp", texture_mesh.tex_coordinates[0]);

	std::vector<pcl::PolygonMeshPtr> p_meshes;
	std::vector<std::string> mesh_ids;
	std::string dir_name = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_decimated_0.900000";

	load_meshes_from_dir(dir_name, p_meshes, mesh_ids);

	//// ********** 
	//// load .ply mesh file
	//pcl::PolygonMesh pmesh;
	//std::string fname = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/cloud_mls/spsr_decimated_0.900000/ptcloud_hd00000380_normals_cleaned.ply";

	//pcl::io::loadPLYFile(fname, pmesh);

	std::string out_dir = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient/";
	// loop thru list of poly meshes
	//for (int idx_mesh = 0; idx_mesh < p_meshes.size(); idx_mesh++) {
	for (int idx_mesh = 0; idx_mesh < 1; idx_mesh++) {
		pcl::PolygonMesh pmesh = *p_meshes[idx_mesh];
		//std::string mesh_id = mesh_ids[idx_mesh];
		boost::filesystem::path path1(mesh_ids[idx_mesh]);
		std::string file_name = path1.stem().string();

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
			return 1;
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
			0, 0.f,					// max # of charts, max stretch param
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
			return 1;
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
			return 1;
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
		pcl::TextureMesh texture_mesh;

		// create PCL vertex buffer equivalent (contains duplicates currently)
		pcl::PointCloud<pcl::PointNormal> xyz;

		//for (int i = 0; i < nVerts; i++) {  // copy newVB to texture_mesh.cloud
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
		//for (int i = 0; i < nTotalVerts; i++) {  // copy newVB to texture_mesh.cloud
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
		//pcl::toPCLPointCloud2(xyz, texture_mesh.cloud);
		pcl::toPCLPointCloud2(*vertices, texture_mesh.cloud);
		//texture_mesh.cloud = pmesh.cloud;

		// create PCL index buffer equivalent
		std::vector<pcl::Vertices> mesh_poly;
		std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >
			mesh_tex;  // indicies correspond to elements in texture_mesh.cloud? // should be same shape as index buffer?

		for (int i = 0; i < ib.size() / 2; i += 3) {  // build each face  // ib.size() / 2
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

		//mesh_material.tex_file = "uv_gradient.jpg";  // should be in same folder as output .obj
		//mesh_material.tex_name = "material_0";

		//std::string tex_path = "C:/Users/maxhu/Desktop/uvatlas_example/test_panoptic.bmp"; // single example
		//std::string tex_path = out_dir + file_name + ".bmp";		// full path
		//std::string tex_path = file_name + ".bmp";				// relative path (assumes .bmp will be in same folder as .obj)
		std::string tex_path = "uv_gradient.png";				
		//generateGradientTexture(tex_path, texture_mesh.tex_coordinates[0]);

		mesh_material.tex_file = tex_path;  // should be in same folder as output .obj
		mesh_material.tex_name = "material_0";

		texture_mesh.tex_materials.push_back(mesh_material);

		// MUST be declared with forward slashes to work correctly
		//std::string obj_path = "C:/Users/maxhu/Desktop/uvatlas_example/test_panoptic.obj"; // single example
		std::string obj_path = out_dir + file_name + ".obj";

		pcl::io::saveOBJFile(obj_path, texture_mesh);

		//// visualize
		//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		//viewer->setPosition(0, 0);
		//viewer->setSize(1173, 732);
		//viewer->addCoordinateSystem(3.0);
		//viewer->addTextureMesh(texture_mesh, "mesh");
		//while (!viewer->wasStopped()) {
		//	viewer->spinOnce(100);
		//	//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		//}


		printf("saved obj to: %s\n", obj_path.c_str());
	}

}
