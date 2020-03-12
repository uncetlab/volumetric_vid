#include <volcap/io/io_usd.h>
#include <pcl/TextureMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/base/vt/array.h>



namespace volcap {
	namespace io {

		UsdExporter::UsdExporter(std::string output_full_path) {
			stage = pxr::UsdStage::CreateNew(output_full_path);
			usdMesh = pxr::UsdGeomMesh::Define(stage, pxr::SdfPath("/TexModel/MeshSequence"));
		}

		void UsdExporter::save() {
			stage->GetRootLayer()->Save();
		}

		void UsdExporter::usdFromTextureMeshes(
			std::vector<pcl::TextureMeshPtr> meshes,
			std::vector<std::string> mesh_names
		) {
			int start_time = 0;
			int full_time = meshes.size();
			stage->SetStartTimeCode(start_time);
			stage->SetEndTimeCode(full_time);

			for (int i = 0; i < meshes.size(); i++) {
				addMesh(*meshes[i], i);
			}

		}

		void UsdExporter::addMesh(pcl::TextureMesh &mesh, int time) {

			std::string primvar_name = "st";

			pxr::VtVec3fArray pts;
			pxr::VtVec3fArray extentArray(2);
			pxr::VtArray<int> faceVertexCounts;
			pxr::VtArray<int> faceVertexIndices;
			pxr::VtVec2fArray texCoordsArray;

			//====== build points
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
			pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
			pts.reserve(cloud->size());

			for (int i = 0; i < cloud->size(); i++) {
				pcl::PointNormal &pcl_v = cloud->points[i];
				pts.push_back(pxr::GfVec3f(pcl_v.x, pcl_v.y, pcl_v.z));
			}

			//====== calculate extent -- not required? skip

			//======= build faces
			std::vector<pcl::Vertices> &submesh = mesh.tex_polygons[0];
			int num_tris = submesh.size();

			for (int tri_idx = 0; tri_idx < submesh.size(); tri_idx++) {
				faceVertexIndices.push_back(submesh[tri_idx].vertices[0]);
				faceVertexIndices.push_back(submesh[tri_idx].vertices[1]);
				faceVertexIndices.push_back(submesh[tri_idx].vertices[2]);
				faceVertexCounts.push_back(3);
			}

			//====== specify uv coords
			std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> &uv_coords = mesh.tex_coordinates[0];

			for (int i = 0; i < uv_coords.size(); i++) {
				Eigen::Vector2f &eigen_uv = uv_coords[i];
				texCoordsArray.push_back(pxr::GfVec2f(eigen_uv(0), eigen_uv(1)));
			}

			//====== define material
			std::string mat_path = "/TexModel/Mat";
			auto usdMaterial = pxr::UsdShadeMaterial::Define(stage, pxr::SdfPath(mat_path));

			// make the surface non - metallic, and somewhat rough
			auto pbrShader = pxr::UsdShadeShader::Define(stage, pxr::SdfPath(mat_path + "/pbrShader"));
			pbrShader.CreateIdAttr(pxr::VtValue(pxr::TfToken("UsdPreviewSurface")));
			pbrShader.CreateInput(pxr::TfToken("roughness"), pxr::SdfValueTypeNames->Float).Set(0.4f);
			pbrShader.CreateInput(pxr::TfToken("metallic"), pxr::SdfValueTypeNames->Float).Set(0.0f);

			usdMaterial.CreateSurfaceOutput().ConnectToSource(pbrShader, pxr::TfToken("surface"));

			// create texture coordinate reader
			auto stReader = pxr::UsdShadeShader::Define(stage, pxr::SdfPath(mat_path + "/stReader"));
			stReader.CreateIdAttr(pxr::VtValue(pxr::TfToken("UsdPrimvarReader_float2")));
			auto stInput = usdMaterial.CreateInput(pxr::TfToken("frame:stPrimvarName"), pxr::SdfValueTypeNames->Token);
			stInput.Set(pxr::TfToken(primvar_name));
			stReader.CreateInput(pxr::TfToken("varname"), pxr::SdfValueTypeNames->Token).ConnectToSource(stInput);

			// diffuse texture
			std::string tex_file = mesh.tex_materials[0].tex_file;
			auto diffuseTextureSampler = pxr::UsdShadeShader::Define(stage, pxr::SdfPath(mat_path + "/diffuseTexture"));
			diffuseTextureSampler.CreateIdAttr(pxr::VtValue(pxr::TfToken("UsdUVTexture")));
			diffuseTextureSampler.CreateInput(pxr::TfToken("file"), pxr::SdfValueTypeNames->Asset)
				.Set(pxr::SdfAssetPath(tex_file), time);  //  "uv_gradient.jpg"

			diffuseTextureSampler.CreateInput(pxr::TfToken("st"), pxr::SdfValueTypeNames->Float2)
				.ConnectToSource(stReader, pxr::TfToken("result"));

			diffuseTextureSampler.CreateOutput(pxr::TfToken("rgb"), pxr::SdfValueTypeNames->Float3);
			pbrShader.CreateInput(pxr::TfToken("diffuseColor"), pxr::SdfValueTypeNames->Color3f)
				.ConnectToSource(diffuseTextureSampler, pxr::TfToken("rgb"));

			//================================
			//     SET THE TIME-SAMPLED ATTRS
			//================================

			usdMesh.GetPointsAttr().Set(pts, time);

			usdMesh.GetFaceVertexCountsAttr().Set(faceVertexCounts, time);
			usdMesh.GetFaceVertexIndicesAttr().Set(faceVertexIndices, time);

			//usdMesh.GetExtentAttr().Set(extentArray, time);  // not required? skip

			// set uv coords
			auto geoPrimApi = pxr::UsdGeomPrimvarsAPI(usdMesh);
			auto texCoords = geoPrimApi.CreatePrimvar(
				pxr::TfToken(primvar_name),  // just a name, can be anything
				pxr::SdfValueTypeNames->TexCoord2fArray,
				pxr::UsdGeomTokens->faceVarying  // pxr::UsdGeomTokens->varying
			);
			texCoords.Set(texCoordsArray, time);

			//Now bind the Material to the board
			pxr::UsdShadeMaterialBindingAPI materialBinding(usdMesh);
			materialBinding.Bind(usdMaterial);
		}
	}
}

