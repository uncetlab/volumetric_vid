#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/base/vt/array.h>

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

void create_seq() {
	auto stage = pxr::UsdStage::CreateNew("HelloMesh.usda");
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

	//======= build points for cube!
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

int main()
{
	//auto stage = pxr::UsdStage::CreateNew("HelloWorld.usda");
	//auto xformPrim = pxr::UsdGeomXform::Define(stage, pxr::SdfPath("/hello"));
	//auto spherePrim = pxr::UsdGeomSphere::Define(stage, pxr::SdfPath("/hello/world"));
	//stage->GetRootLayer()->Save();

	////xformPrim.
	//return 0;

	create_seq();
}