#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/base/vt/array.h>

// get the project directory from preprocessor set in the top-level CMakeLists.txt
const std::string PROJECT_DIR = _PROJECT_DIR;

// adds cube to mesh at time `time`
void add_cube(pxr::UsdStageRefPtr &stage, pxr::UsdGeomMesh &mesh, int time=0) {
	std::string primvar_name = "st";

	pxr::VtVec3fArray pts;
	pxr::VtVec3fArray extentArray(2);
	pxr::VtArray<int> faceVertexCounts;
	pxr::VtArray<int> faceVertexIndices;
	pxr::VtVec2fArray texCoordsArray;

	//======= build points for cube!
	pts.reserve(8);
	pts.push_back(pxr::GfVec3f(0, 0, 0));		// 0
	pts.push_back(pxr::GfVec3f(100, 0, 0));		// 1
	pts.push_back(pxr::GfVec3f(100, 0, 100));	// 2
	pts.push_back(pxr::GfVec3f(0, 0, 100));		// 3
	pts.push_back(pxr::GfVec3f(0, 100, 0));		// 4
	pts.push_back(pxr::GfVec3f(100, 100, 0));	// 5
	pts.push_back(pxr::GfVec3f(100, 100, 100));	// 6
	pts.push_back(pxr::GfVec3f(0, 100, 100));	// 7

	//====== calculate extent -- not actually required? it still works is usdview without extent
	pxr::GfRange3f extent;
	for (const auto& pt : pts) {
		extent.UnionWith(pt);
	}
	extentArray[0] = extent.GetMin();
	extentArray[1] = extent.GetMax();

	//======= build faces

	// face 1
	faceVertexIndices.push_back(0);
	faceVertexIndices.push_back(1);
	faceVertexIndices.push_back(2);
	faceVertexIndices.push_back(3);
	faceVertexCounts.push_back(4);

	// face 2
	faceVertexIndices.push_back(0);
	faceVertexIndices.push_back(3);
	faceVertexIndices.push_back(7);
	faceVertexIndices.push_back(4);
	faceVertexCounts.push_back(4);

	// face 3
	faceVertexIndices.push_back(3);
	faceVertexIndices.push_back(2);
	faceVertexIndices.push_back(6);
	faceVertexIndices.push_back(7);
	faceVertexCounts.push_back(4);

	// face 4
	faceVertexIndices.push_back(2);
	faceVertexIndices.push_back(1);
	faceVertexIndices.push_back(5);
	faceVertexIndices.push_back(6);
	faceVertexCounts.push_back(4);

	// face 5
	faceVertexIndices.push_back(1);
	faceVertexIndices.push_back(0);
	faceVertexIndices.push_back(4);
	faceVertexIndices.push_back(5);
	faceVertexCounts.push_back(4);

	// face 6
	faceVertexIndices.push_back(4);
	faceVertexIndices.push_back(5);
	faceVertexIndices.push_back(6);
	faceVertexIndices.push_back(7);
	faceVertexCounts.push_back(4);

	//====== specify uv coords

	// 1
	texCoordsArray.push_back(pxr::GfVec2f(0, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 1));
	texCoordsArray.push_back(pxr::GfVec2f(0, 1));

	// 2
	texCoordsArray.push_back(pxr::GfVec2f(0, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 1));
	texCoordsArray.push_back(pxr::GfVec2f(0, 1));

	// 3
	texCoordsArray.push_back(pxr::GfVec2f(0, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 1));
	texCoordsArray.push_back(pxr::GfVec2f(0, 1));

	// 4
	texCoordsArray.push_back(pxr::GfVec2f(0, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 1));
	texCoordsArray.push_back(pxr::GfVec2f(0, 1));

	// 5
	texCoordsArray.push_back(pxr::GfVec2f(0, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 1));
	texCoordsArray.push_back(pxr::GfVec2f(0, 1));

	// 6
	texCoordsArray.push_back(pxr::GfVec2f(0, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 1));
	texCoordsArray.push_back(pxr::GfVec2f(0, 1));

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
	auto diffuseTextureSampler = pxr::UsdShadeShader::Define(stage, pxr::SdfPath(mat_path + "/diffuseTexture"));
	diffuseTextureSampler.CreateIdAttr(pxr::VtValue(pxr::TfToken("UsdUVTexture")));
	diffuseTextureSampler.CreateInput(pxr::TfToken("file"), pxr::SdfValueTypeNames->Asset)
		.Set(pxr::SdfAssetPath(PROJECT_DIR + "/demo_data/Crate.jpg"), time);  //  "Crate.jpg"

	diffuseTextureSampler.CreateInput(pxr::TfToken("st"), pxr::SdfValueTypeNames->Float2)
		.ConnectToSource(stReader, pxr::TfToken("result"));

	diffuseTextureSampler.CreateOutput(pxr::TfToken("rgb"), pxr::SdfValueTypeNames->Float3);
	pbrShader.CreateInput(pxr::TfToken("diffuseColor"), pxr::SdfValueTypeNames->Color3f)
		.ConnectToSource(diffuseTextureSampler, pxr::TfToken("rgb"));

	//================================
	//     SET THE TIME-SAMPLED ATTRS
	//================================

	mesh.GetPointsAttr().Set(pts, time);

	mesh.GetFaceVertexCountsAttr().Set(faceVertexCounts, time);
	mesh.GetFaceVertexIndicesAttr().Set(faceVertexIndices, time);

	mesh.GetExtentAttr().Set(extentArray, time);

	// set uv coords
	auto geoPrimApi = pxr::UsdGeomPrimvarsAPI(mesh);
	auto texCoords = geoPrimApi.CreatePrimvar(
		pxr::TfToken(primvar_name),  // just a name, can be anything
		pxr::SdfValueTypeNames->TexCoord2fArray,
		pxr::UsdGeomTokens->faceVarying  // pxr::UsdGeomTokens->varying
	);
	texCoords.Set(texCoordsArray, time);

	//Now bind the Material to the board
	pxr::UsdShadeMaterialBindingAPI materialBinding(mesh);
	materialBinding.Bind(usdMaterial);
}

// adds pyramid to mesh at time `time`
void add_pyramid(pxr::UsdStageRefPtr &stage, pxr::UsdGeomMesh &mesh, int time = 0) {
	std::string primvar_name = "st";

	pxr::VtVec3fArray pts;
	pxr::VtVec3fArray extentArray(2);
	pxr::VtArray<int> faceVertexCounts;
	pxr::VtArray<int> faceVertexIndices;
	pxr::VtVec2fArray texCoordsArray;

	//======= build points for pyramid!

	pts.reserve(4);
	pts.push_back(pxr::GfVec3f(0, 0, 0));		// 0
	pts.push_back(pxr::GfVec3f(100, 0, 50));	// 1
	pts.push_back(pxr::GfVec3f(0, 0, 100));		// 2
	pts.push_back(pxr::GfVec3f(50, 100, 50));	// 3

	//====== calculate extent
	pxr::GfRange3f extent1;
	for (const auto& pt : pts) {
		extent1.UnionWith(pt);
	}
	extentArray[0] = extent1.GetMin();
	extentArray[1] = extent1.GetMax();

	//======= build faces

	// face 1
	faceVertexIndices.push_back(0);
	faceVertexIndices.push_back(1);
	faceVertexIndices.push_back(2);
	faceVertexCounts.push_back(3);

	// face 2
	faceVertexIndices.push_back(0);
	faceVertexIndices.push_back(3);
	faceVertexIndices.push_back(2);
	faceVertexCounts.push_back(3);

	// face 3
	faceVertexIndices.push_back(1);
	faceVertexIndices.push_back(3);
	faceVertexIndices.push_back(2);
	faceVertexCounts.push_back(3);

	// face 4
	faceVertexIndices.push_back(2);
	faceVertexIndices.push_back(3);
	faceVertexIndices.push_back(0);
	faceVertexCounts.push_back(3);

	//====== specify uv coords

	// 1
	texCoordsArray.push_back(pxr::GfVec2f(0, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 1));

	// 2
	texCoordsArray.push_back(pxr::GfVec2f(0, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 1));

	// 3
	texCoordsArray.push_back(pxr::GfVec2f(0, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 1));

	// 4
	texCoordsArray.push_back(pxr::GfVec2f(0, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 0));
	texCoordsArray.push_back(pxr::GfVec2f(1, 1));

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
	auto diffuseTextureSampler = pxr::UsdShadeShader::Define(stage, pxr::SdfPath(mat_path + "/diffuseTexture"));
	diffuseTextureSampler.CreateIdAttr(pxr::VtValue(pxr::TfToken("UsdUVTexture")));
	diffuseTextureSampler.CreateInput(pxr::TfToken("file"), pxr::SdfValueTypeNames->Asset)
		.Set(pxr::SdfAssetPath(PROJECT_DIR + "/demo_data/uv_gradient.jpg"), time);  //  "uv_gradient.jpg"

	diffuseTextureSampler.CreateInput(pxr::TfToken("st"), pxr::SdfValueTypeNames->Float2)
		.ConnectToSource(stReader, pxr::TfToken("result"));

	diffuseTextureSampler.CreateOutput(pxr::TfToken("rgb"), pxr::SdfValueTypeNames->Float3);
	pbrShader.CreateInput(pxr::TfToken("diffuseColor"), pxr::SdfValueTypeNames->Color3f)
		.ConnectToSource(diffuseTextureSampler, pxr::TfToken("rgb"));

	//================================
	//     SET THE TIME-SAMPLED ATTRS
	//================================

	mesh.GetPointsAttr().Set(pts, time);

	mesh.GetFaceVertexCountsAttr().Set(faceVertexCounts, time);
	mesh.GetFaceVertexIndicesAttr().Set(faceVertexIndices, time);

	mesh.GetExtentAttr().Set(extentArray, time);

	// set uv coords
	auto geoPrimApi = pxr::UsdGeomPrimvarsAPI(mesh);
	auto texCoords = geoPrimApi.CreatePrimvar(
		pxr::TfToken(primvar_name),  // just a name, can be anything
		pxr::SdfValueTypeNames->TexCoord2fArray,
		pxr::UsdGeomTokens->faceVarying  // pxr::UsdGeomTokens->varying
	);
	texCoords.Set(texCoordsArray, time);

	//Now bind the Material to the board
	pxr::UsdShadeMaterialBindingAPI materialBinding(mesh);
	materialBinding.Bind(usdMaterial);
}

void demo_create_seq() {
	auto stage = pxr::UsdStage::CreateNew(PROJECT_DIR + "/demos/demo_output/TimeSampleCratePyramid.usda");
	int start_time = 0;
	int half_time = 24;
	stage->SetStartTimeCode(start_time);
	stage->SetEndTimeCode(half_time*2);

	auto usdMesh = pxr::UsdGeomMesh::Define(stage, pxr::SdfPath("/TexModel/MeshSequence"));

	//================================
	//         TIME SAMPLE 1: CUBE
	//================================

	add_cube(stage, usdMesh, start_time);

	//================================
	//         TIME SAMPLE 2: PYRAMID
	//================================

	add_pyramid(stage, usdMesh, half_time);

	//================================
	//     SAVE
	//================================

	stage->GetRootLayer()->Save();
}

void demo_create_crate() {
	auto stage = pxr::UsdStage::CreateNew(PROJECT_DIR + "/demos/demo_output/Crate.usda");
	auto usdMesh = pxr::UsdGeomMesh::Define(stage, pxr::SdfPath("/TexModel/CrateMesh"));

	add_cube(stage, usdMesh);

	stage->GetRootLayer()->Save();
}

int main()
{

	// creates a single textured crate
	demo_create_crate();

	//// creates a 2 second sequence, 1st sec is textured crate, 2nd sec is textured pyramid
	//demo_create_seq();
}
