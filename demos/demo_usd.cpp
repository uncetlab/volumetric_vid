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

void build_cube(
	pxr::VtVec3fArray &pts, pxr::VtVec3fArray &extentArray, pxr::VtArray<int> &faceVertexCounts, pxr::VtArray<int> &faceVertexIndices
) {

	//======= build points for cube!
	pts.reserve(8);
	pts.push_back(pxr::GfVec3f(-50, 0, 50));	// 0
	pts.push_back(pxr::GfVec3f(50, 0, 50));		// 1
	pts.push_back(pxr::GfVec3f(50, 100, 50));	// 2
	pts.push_back(pxr::GfVec3f(-50, 100, 50));	// 3
	pts.push_back(pxr::GfVec3f(-50, 0, -50));	// 4
	pts.push_back(pxr::GfVec3f(50, 0, -50));	// 5
	pts.push_back(pxr::GfVec3f(50, 100, -50));	// 6
	pts.push_back(pxr::GfVec3f(-50, 100, -50));	// 7

	//====== calculate extent
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
}

void build_pyramid(
	pxr::VtVec3fArray &pts, pxr::VtVec3fArray &extentArray, pxr::VtArray<int> &faceVertexCounts, pxr::VtArray<int> &faceVertexIndices
) {
	//======= build points for pyramid!

	pts.reserve(4);
	pts.push_back(pxr::GfVec3f(0, 0, 0));		// 0
	pts.push_back(pxr::GfVec3f(50, 0, 0));		// 1
	pts.push_back(pxr::GfVec3f(25, 50, 0));		// 2
	pts.push_back(pxr::GfVec3f(25, 25, 50));	// 3

	// Usd currently requires an extent, somewhat unfortunately.
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

}

void add_cube_material(pxr::UsdGeomMesh &mesh, pxr::UsdStageRefPtr &stage) {

	//====== add texture coords
	auto geoPrimApi = pxr::UsdGeomPrimvarsAPI(mesh);
	auto texCoords = geoPrimApi.CreatePrimvar(
		pxr::TfToken("fdsa"),  // just a name, can be anything
		pxr::SdfValueTypeNames->TexCoord2fArray,
		pxr::UsdGeomTokens->faceVarying  // pxr::UsdGeomTokens->varying
	);

	pxr::VtVec2fArray texCoordsArray;

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

	texCoords.Set(texCoordsArray);

	//====== define material
	auto usdMaterial = pxr::UsdShadeMaterial::Define(stage, pxr::SdfPath("/root/crateMat"));

	// make the surface non - metallic, and somewhat rough
	auto pbrShader = pxr::UsdShadeShader::Define(stage, pxr::SdfPath("/root/crateMat/pbrShader"));
	pbrShader.CreateIdAttr(pxr::VtValue(pxr::TfToken("UsdPreviewSurface")));
	pbrShader.CreateInput(pxr::TfToken("roughness"), pxr::SdfValueTypeNames->Float).Set(0.4f);
	pbrShader.CreateInput(pxr::TfToken("metallic"), pxr::SdfValueTypeNames->Float).Set(0.0f);

	usdMaterial.CreateSurfaceOutput().ConnectToSource(pbrShader, pxr::TfToken("surface"));

	//create texture coordinate reader
	auto stReader = pxr::UsdShadeShader::Define(stage, pxr::SdfPath("/root/crateMat/stReader"));
	stReader.CreateIdAttr(pxr::VtValue(pxr::TfToken("UsdPrimvarReader_float2")));
	auto stInput = usdMaterial.CreateInput(pxr::TfToken("frame:stPrimvarName"), pxr::SdfValueTypeNames->Token);
	stInput.Set(pxr::TfToken("fdsa"));
	stReader.CreateInput(pxr::TfToken("varname"), pxr::SdfValueTypeNames->Token).ConnectToSource(stInput);

	//diffuse texture
	auto diffuseTextureSampler = pxr::UsdShadeShader::Define(stage, pxr::SdfPath("/root/crateMat/diffuseTexture"));
	diffuseTextureSampler.CreateIdAttr(pxr::VtValue(pxr::TfToken("UsdUVTexture")));
	diffuseTextureSampler.CreateInput(pxr::TfToken("file"), pxr::SdfValueTypeNames->Asset)
		.Set(pxr::SdfAssetPath(PROJECT_DIR + "/demo_data/Crate.jpg"));  //  "Crate.jpg"

	diffuseTextureSampler.CreateInput(pxr::TfToken("st"), pxr::SdfValueTypeNames->Float2)
		.ConnectToSource(stReader, pxr::TfToken("result"));

	diffuseTextureSampler.CreateOutput(pxr::TfToken("rgb"), pxr::SdfValueTypeNames->Float3);
	pbrShader.CreateInput(pxr::TfToken("diffuseColor"), pxr::SdfValueTypeNames->Color3f)
		.ConnectToSource(diffuseTextureSampler, pxr::TfToken("rgb"));

	//Now bind the Material to the board
	pxr::UsdShadeMaterialBindingAPI materialBinding(mesh);
	materialBinding.Bind(usdMaterial);
}



void demo_create_seq() {
	auto stage = pxr::UsdStage::CreateNew("CratePyramidTimeSample.usda");
	int start_time = 0;
	int half_time = 24;
	stage->SetStartTimeCode(start_time);
	stage->SetEndTimeCode(half_time*2);

	auto usdMesh = pxr::UsdGeomMesh::Define(stage, pxr::SdfPath("/hello/mesh"));

	//================================
	//         TIME SAMPLE 1: CUBE
	//================================

	pxr::VtVec3fArray usdPoints0;
	pxr::VtVec3fArray extentArray0(2);
	pxr::VtArray<int> faceVertexCounts0, faceVertexIndices0;
	build_cube(usdPoints0, extentArray0, faceVertexCounts0, faceVertexIndices0);

	//================================
	//         TIME SAMPLE 2: PYRAMID
	//================================

	pxr::VtVec3fArray usdPoints1;
	pxr::VtVec3fArray extentArray1(2);
	pxr::VtArray<int> faceVertexCounts1, faceVertexIndices1;
	build_pyramid(usdPoints1, extentArray1, faceVertexCounts1, faceVertexIndices1);

	//================================
	//     SET THE TIME-SAMPLED ATTRS
	//================================
	
	usdMesh.GetPointsAttr().Set(usdPoints0, start_time);
	usdMesh.GetPointsAttr().Set(usdPoints1, half_time);

	usdMesh.GetFaceVertexCountsAttr().Set(faceVertexCounts0, start_time);
	usdMesh.GetFaceVertexCountsAttr().Set(faceVertexCounts1, half_time);
	usdMesh.GetFaceVertexIndicesAttr().Set(faceVertexIndices0, start_time);
	usdMesh.GetFaceVertexIndicesAttr().Set(faceVertexIndices1, half_time);

	usdMesh.GetExtentAttr().Set(extentArray0, start_time);
	usdMesh.GetExtentAttr().Set(extentArray1, half_time);

	stage->GetRootLayer()->Save();
}

void demo_create_crate() {
	auto stage = pxr::UsdStage::CreateNew("Crate.usda");

	//====== define mesh
	auto usdMesh = pxr::UsdGeomMesh::Define(stage, pxr::SdfPath("/root/crate"));

	pxr::VtVec3fArray usdPoints0;
	pxr::VtVec3fArray extentArray0(2);
	pxr::VtArray<int> faceVertexCounts0, faceVertexIndices0;
	build_cube(usdPoints0, extentArray0, faceVertexCounts0, faceVertexIndices0);

	// set attrs
	usdMesh.GetPointsAttr().Set(usdPoints0);
	usdMesh.GetFaceVertexCountsAttr().Set(faceVertexCounts0);
	usdMesh.GetFaceVertexIndicesAttr().Set(faceVertexIndices0);
	usdMesh.GetExtentAttr().Set(extentArray0);

	//====== apply material
	add_cube_material(usdMesh, stage);

	stage->GetRootLayer()->Save();
}

int main()
{

	demo_create_crate();

	//demo_create_seq();
}