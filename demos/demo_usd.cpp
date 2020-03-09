#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/base/vt/array.h>


void create_seq() {
	auto stage = pxr::UsdStage::CreateNew("HelloMesh.usda");
	stage->SetStartTimeCode(0);
	stage->SetEndTimeCode(48);

	auto usdMesh = pxr::UsdGeomMesh::Define(stage, pxr::SdfPath("/hello/mesh"));

	//================================
	//         TIME SAMPLE 1: CUBE
	//================================

	//======= build points for cube!
	pxr::VtVec3fArray usdPoints0;

	usdPoints0.reserve(8);
	usdPoints0.push_back(pxr::GfVec3f(-50, 0, 50));		// 0
	usdPoints0.push_back(pxr::GfVec3f(50, 0, 50));		// 1
	usdPoints0.push_back(pxr::GfVec3f(50, 100, 50));	// 2
	usdPoints0.push_back(pxr::GfVec3f(-50, 100, 50));	// 3
	usdPoints0.push_back(pxr::GfVec3f(-50, 0, -50));	// 4
	usdPoints0.push_back(pxr::GfVec3f(50, 0, -50));		// 5
	usdPoints0.push_back(pxr::GfVec3f(50, 100, -50));	// 6
	usdPoints0.push_back(pxr::GfVec3f(-50, 100, -50));	// 7

	// Usd currently requires an extent, somewhat unfortunately.
	pxr::GfRange3f extent0;
	for (const auto& pt : usdPoints0) {
		extent0.UnionWith(pt);
	}
	pxr::VtVec3fArray extentArray0(2);
	extentArray0[0] = extent0.GetMin();
	extentArray0[1] = extent0.GetMax();

	//======= build faces

	//store a vector with number of sides per polygon
	//in the test model there are only triangles so this is going to be 3, 3, 3, ... for the number of polygons
	pxr::VtArray<int> faceVertexCounts0, faceVertexIndices0;

	// face 1
	faceVertexIndices0.push_back(0);
	faceVertexIndices0.push_back(1);
	faceVertexIndices0.push_back(2);
	faceVertexIndices0.push_back(3);
	faceVertexCounts0.push_back(4);

	// face 2
	faceVertexIndices0.push_back(0);
	faceVertexIndices0.push_back(3);
	faceVertexIndices0.push_back(7);
	faceVertexIndices0.push_back(4);
	faceVertexCounts0.push_back(4);

	// face 3
	faceVertexIndices0.push_back(3);
	faceVertexIndices0.push_back(2);
	faceVertexIndices0.push_back(6);
	faceVertexIndices0.push_back(7);
	faceVertexCounts0.push_back(4);

	// face 4
	faceVertexIndices0.push_back(2);
	faceVertexIndices0.push_back(1);
	faceVertexIndices0.push_back(5);
	faceVertexIndices0.push_back(6);
	faceVertexCounts0.push_back(4);

	// face 5
	faceVertexIndices0.push_back(1);
	faceVertexIndices0.push_back(0);
	faceVertexIndices0.push_back(4);
	faceVertexIndices0.push_back(5);
	faceVertexCounts0.push_back(4);

	// face 6
	faceVertexIndices0.push_back(4);
	faceVertexIndices0.push_back(5);
	faceVertexIndices0.push_back(6);
	faceVertexIndices0.push_back(7);
	faceVertexCounts0.push_back(4);

	//================================
	//         TIME SAMPLE 2: PYRAMID
	//================================

	//======= build points for cube!
	pxr::VtVec3fArray usdPoints1;

	usdPoints1.reserve(4);
	usdPoints1.push_back(pxr::GfVec3f(0, 0, 0));		// 0
	usdPoints1.push_back(pxr::GfVec3f(50, 0, 0));		// 1
	usdPoints1.push_back(pxr::GfVec3f(25, 50, 0));		// 2
	usdPoints1.push_back(pxr::GfVec3f(25, 25, 50));		// 3

	// Usd currently requires an extent, somewhat unfortunately.
	pxr::GfRange3f extent1;
	for (const auto& pt : usdPoints1) {
		extent1.UnionWith(pt);
	}
	pxr::VtVec3fArray extentArray1(2);
	extentArray1[0] = extent1.GetMin();
	extentArray1[1] = extent1.GetMax();

	//======= build faces

	//store a vector with number of sides per polygon
	//in the test model there are only triangles so this is going to be 3, 3, 3, ... for the number of polygons
	pxr::VtArray<int> faceVertexCounts1, faceVertexIndices1;

	// face 1
	faceVertexIndices1.push_back(0);
	faceVertexIndices1.push_back(1);
	faceVertexIndices1.push_back(2);
	faceVertexCounts1.push_back(3);

	// face 2
	faceVertexIndices1.push_back(0);
	faceVertexIndices1.push_back(3);
	faceVertexIndices1.push_back(2);
	faceVertexCounts1.push_back(3);

	// face 3
	faceVertexIndices1.push_back(1);
	faceVertexIndices1.push_back(3);
	faceVertexIndices1.push_back(2);
	faceVertexCounts1.push_back(3);

	// face 4
	faceVertexIndices1.push_back(2);
	faceVertexIndices1.push_back(3);
	faceVertexIndices1.push_back(0);
	faceVertexCounts1.push_back(3);

	//================================
	//     SET THE TIME-SAMPLED ATTRS
	//================================
	int start_time = 0;
	int half_time = 24;

	//usdMesh.GetPointsAttr().GetTimeSamples()
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