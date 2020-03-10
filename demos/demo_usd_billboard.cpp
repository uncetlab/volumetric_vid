#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/base/vt/array.h>

using namespace pxr;

void CreateBillboard()
{
	UsdStageRefPtr stage = UsdStage::CreateInMemory();

	auto modelRoot = UsdGeomXform::Define(stage, SdfPath("/TexModel"));
	//TF_VERIFY(modelRoot,
	//	"Failed to create prim at %s",
	//	-hide quoted text -
	//	"/TexModel");

	UsdGeomXformCommonAPI transform(modelRoot);
	transform.SetTranslate(GfVec3f(4, 5, 6));

	//A simple card with same proportions as the texture we will map
	auto billboard = UsdGeomMesh::Define(stage, SdfPath("/TexModel/card"));
	VtVec3fArray billboardPoints;
	billboardPoints.push_back(GfVec3f(-430, -145, 0));
	billboardPoints.push_back(GfVec3f(430, -145, 0));
	billboardPoints.push_back(GfVec3f(430, 145, 0));
	billboardPoints.push_back(GfVec3f(-430, 145, 0));
	billboard.CreatePointsAttr(VtValue(billboardPoints));

	VtArray<int> faceVertexCounts;
	faceVertexCounts.push_back(4);
	billboard.CreateFaceVertexCountsAttr(VtValue(faceVertexCounts));

	VtArray<int> faceVertexIndices;
	faceVertexIndices.push_back(0);
	faceVertexIndices.push_back(1);
	faceVertexIndices.push_back(2);
	faceVertexIndices.push_back(3);
	billboard.CreateFaceVertexIndicesAttr(VtValue(faceVertexIndices));

	// Set extent.
	VtVec3fArray extent;
	UsdGeomBoundable::ComputeExtentFromPlugins(billboard, UsdTimeCode::Default(), &extent);
	billboard.CreateExtentAttr(VtValue(extent));

	auto texCoords = billboard.CreatePrimvar(TfToken("st"),
		SdfValueTypeNames->TexCoord2fArray,
		UsdGeomTokens->varying);

	VtVec2fArray texCoordsArray;
	texCoordsArray.push_back(GfVec2f(0, 0));
	texCoordsArray.push_back(GfVec2f(1, 0));
	texCoordsArray.push_back(GfVec2f(1, 1));
	texCoordsArray.push_back(GfVec2f(0, 1));
	texCoords.Set(texCoordsArray);

	//material creation
	/*********************************/
	//Now make a Material that contains a PBR preview surface, a texture reader,
	//and a primvar reader to fetch the texture coordinate from the geometry
	auto material = UsdShadeMaterial::Define(stage, SdfPath("/TexModel/card/boardMat"));
	auto materialInput = material.CreateInput(TfToken("frame:stPrimvarName"), SdfValueTypeNames->Token);
	materialInput.Set(TfToken("st"));

	//Create surface, and connect the Material's surface output to the surface 
	//shader.Make the surface non - metallic, and somewhat rough, so it doesn't
	//glare in usdview's simple camera light setup.
	auto pbrShader = UsdShadeShader::Define(stage, SdfPath("/TexModel/card/boardMat/pbrShader"));
	pbrShader.CreateIdAttr(VtValue(TfToken("UsdPreviewSurface")));

	//Provide fallback values for other PBR inputs in release 18.09
	pbrShader.CreateInput(TfToken("clearcoat"), SdfValueTypeNames->Float).Set(0.0f);
	pbrShader.CreateInput(TfToken("clearcoatRoughness"), SdfValueTypeNames->Float).Set(0.01f);
	pbrShader.CreateInput(TfToken("emissiveColor"), SdfValueTypeNames->Float3).Set(GfVec3f(0.f, 0.f, 0.f));
	pbrShader.CreateInput(TfToken("ior"), SdfValueTypeNames->Float).Set(1.5f);
	pbrShader.CreateInput(TfToken("normal"), SdfValueTypeNames->Float3).Set(GfVec3f(0.f, 0.f, 1.f));
	pbrShader.CreateInput(TfToken("occlusion"), SdfValueTypeNames->Float).Set(1.f);
	pbrShader.CreateInput(TfToken("opacity"), SdfValueTypeNames->Float).Set(1.f);
	pbrShader.CreateInput(TfToken("specularColor"), SdfValueTypeNames->Float3).Set(GfVec3f(0.f, 0.f, 0.f));
	pbrShader.CreateInput(TfToken("useSpecularWorkflow"), SdfValueTypeNames->Int).Set(0);

	pbrShader.CreateInput(TfToken("roughness"), SdfValueTypeNames->Float).Set(0.4f);
	pbrShader.CreateInput(TfToken("metallic"), SdfValueTypeNames->Float).Set(0.0f);

	material.CreateSurfaceOutput().ConnectToSource(pbrShader, TfToken("surface"));

	//create texture coordinate reader
	auto stReader = UsdShadeShader::Define(stage, SdfPath("/TexModel/card/boardMat/stReader"));
	stReader.CreateIdAttr(VtValue(TfToken("UsdPrimvarReader_float2")));
	//Note here we are connecting the shader's input to the material's
	//"public interface" attribute. This allows users to change the primvar name
	//on the material itself without drilling inside to examine shader nodes.
	stReader.CreateInput(TfToken("varname"), SdfValueTypeNames->Token).ConnectToSource(materialInput);

	//diffuse texture
	auto diffuseTextureSampler = UsdShadeShader::Define(stage, SdfPath("/TexModel/card/boardMat/diffuseTexture"));
	diffuseTextureSampler.CreateIdAttr(VtValue(TfToken("UsdUVTexture")));
	diffuseTextureSampler.CreateInput(TfToken("file"), SdfValueTypeNames->Asset).Set(SdfAssetPath("USDLogoLrg.png"));
	diffuseTextureSampler.CreateInput(TfToken("st"), SdfValueTypeNames->Float2).ConnectToSource(stReader, TfToken("result"));
	diffuseTextureSampler.CreateOutput(TfToken("rgb"), SdfValueTypeNames->Float3);
	pbrShader.CreateInput(TfToken("diffuseColor"), SdfValueTypeNames->Color3f).ConnectToSource(diffuseTextureSampler, TfToken("rgb"));

	//Now bind the Material to the board
	UsdShadeMaterialBindingAPI materialBinding(billboard);
	materialBinding.Bind(material);
	/*********************************/


	stage->GetRootLayer()->Export("Billboard.usda");
};

int main()
{
	CreateBillboard();
}
