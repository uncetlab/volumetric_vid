#include <fbxsdk.h>
#include <vector>


// declare globals
FbxManager*   gSdkManager = NULL;
FbxScene*        gScene = NULL;
FbxFileTexture*  gTexture = NULL;
FbxSurfacePhong* gMaterial = NULL;

#ifdef IOS_REF
	#undef  IOS_REF
	#define IOS_REF (*(pSdkManager->GetIOSettings()))
#endif

bool InitializeSdkObjects(FbxManager*& pManager, FbxScene*& pScene)
{
	//The first thing to do is to create the FBX Manager which is the object allocator for almost all the classes in the SDK
	pManager = FbxManager::Create();
	if (!pManager)
	{
		FBXSDK_printf("Error: Unable to create FBX Manager!\n");
		exit(1);
	}
	else FBXSDK_printf("Autodesk FBX SDK version %s\n", pManager->GetVersion());

	//Create an IOSettings object. This object holds all import/export settings.
	FbxIOSettings* ios = FbxIOSettings::Create(pManager, IOSROOT);
	pManager->SetIOSettings(ios);

	//Create an FBX scene. This object holds most objects imported/exported from/to files.
	pScene = FbxScene::Create(pManager, "My Scene");
	if (!pScene)
	{
		FBXSDK_printf("Error: Unable to create FBX scene!\n");
		exit(1);
	}
	return true;
}

// Create a global texture for cube.
void CreateTexture(FbxScene* pScene)
{
	gTexture = FbxFileTexture::Create(pScene, "Diffuse Texture");

	// Resource file must be in the application's directory.
	//FbxString lTexPath = gAppPath ? *gAppPath + "\\Crate.jpg" : "";
	//FbxString lTexPath = "C:/Program Files/Autodesk/FBX/FBX SDK/2020.0.1/samples/UI Examples/CubeCreator/Crate.jpg";
	FbxString lTexPath = "C:/Users/maxhu/Desktop/uvatlas_example/fbx_sdk/Crate.jpg"; // this texture gets embedded in the binary .fbx file
	//FbxString lTexRelPath = "Crate.jpg";

	// Set texture properties.
	gTexture->SetFileName(lTexPath.Buffer());  // must pass in an absolute path
	//bool setRelFileName = gTexture->SetRelativeFileName(lTexRelPath.Buffer());  // relative path takes preference over absolute?
	gTexture->SetTextureUse(FbxTexture::eStandard);
	gTexture->SetMappingType(FbxTexture::eUV);
	gTexture->SetMaterialUse(FbxFileTexture::eModelMaterial);
	gTexture->SetSwapUV(false);
	gTexture->SetTranslation(0.0, 0.0);
	gTexture->SetScale(1.0, 1.0);
	gTexture->SetRotation(0.0, 0.0);
}

// Create global material for cube.
void CreateMaterial(FbxScene* pScene)
{
	FbxString lMaterialName = "material";
	FbxString lShadingName = "Phong";
	FbxDouble3 lBlack(0.0, 0.0, 0.0);
	FbxDouble3 lRed(1.0, 0.0, 0.0);
	FbxDouble3 lDiffuseColor(0.75, 0.75, 0.0);
	gMaterial = FbxSurfacePhong::Create(pScene, lMaterialName.Buffer());

	// Generate primary and secondary colors.
	gMaterial->Emissive.Set(lBlack);
	gMaterial->Ambient.Set(lRed);
	gMaterial->Diffuse.Set(lDiffuseColor);
	gMaterial->TransparencyFactor.Set(40.5);
	gMaterial->ShadingModel.Set(lShadingName);
	gMaterial->Shininess.Set(0.5);

	// the texture need to be connected to the material on the corresponding property
	if (gTexture)
		gMaterial->Diffuse.ConnectSrcObject(gTexture);
}

// to create a basic scene
bool CreateScene()
{
	// Initialize the FbxManager and the FbxScene
	if (InitializeSdkObjects(gSdkManager, gScene) == false)
	{
		return false;
	}

	//// set the animation stack and use the unique AnimLayer to support all the animation
	//FbxAnimStack* lAnimStack = FbxAnimStack::Create(gScene, "Animation stack camera animation");
	//gAnimLayer = FbxAnimLayer::Create(gScene, "Base Layer");
	//lAnimStack->AddMember(gAnimLayer);

	//// create a marker
	//FbxNode* lMarker = CreateMarker(gScene, "Marker");

	//// create a camera
	//FbxNode* lCamera = CreateCamera(gScene, "Camera");

	// create a single texture shared by all cubes
	CreateTexture(gScene);

	// create a material shared by all faces of all cubes
	CreateMaterial(gScene);

	//// set the camera point of interest on the marker
	//SetCameraPointOfInterest(lCamera, lMarker);

	//// set the marker position
	//SetMarkerDefaultPosition(lMarker);

	//// set the camera position
	//SetCameraDefaultPosition(lCamera);

	//// animate the camera
	//AnimateCamera(lCamera, gAnimLayer);

	//// build a minimum scene graph
	//FbxNode* lRootNode = gScene->GetRootNode();
	//lRootNode->AddChild(lMarker);
	//lRootNode->AddChild(lCamera);

	//// set camera switcher as the default camera
	//gScene->GetGlobalSettings().SetDefaultCamera((char *)lCamera->GetName());

	return true;
}

// to save a scene to a FBX file
bool SaveScene(FbxManager* pSdkManager, FbxDocument* pScene, const char* pFilename, int pFileFormat, bool pEmbedMedia)
{
	if (pSdkManager == NULL) return false;
	if (pScene == NULL) return false;
	if (pFilename == NULL) return false;

	bool lStatus = true;

	// Create an exporter.
	FbxExporter* lExporter = FbxExporter::Create(pSdkManager, "");

	if (pFileFormat < 0 || pFileFormat >= pSdkManager->GetIOPluginRegistry()->GetWriterFormatCount())
	{
		// Write in fall back format if pEmbedMedia is true
		pFileFormat = pSdkManager->GetIOPluginRegistry()->GetNativeWriterFormat();

		if (!pEmbedMedia)
		{
			//Try to export in ASCII if possible
			int lFormatIndex, lFormatCount = pSdkManager->GetIOPluginRegistry()->GetWriterFormatCount();

			for (lFormatIndex = 0; lFormatIndex < lFormatCount; lFormatIndex++)
			{
				if (pSdkManager->GetIOPluginRegistry()->WriterIsFBX(lFormatIndex))
				{
					FbxString lDesc = pSdkManager->GetIOPluginRegistry()->GetWriterFormatDescription(lFormatIndex);
					char *lASCII = "ascii";
					if (lDesc.Find(lASCII) >= 0)
					{
						pFileFormat = lFormatIndex;
						break;
					}
				}
			}
		}
	}

	// Initialize the exporter by providing a filename.
	if (lExporter->Initialize(pFilename, pFileFormat, pSdkManager->GetIOSettings()) == false)
	{
		return false;
	}

	// Set the export states. By default, the export states are always set to 
	// true except for the option eEXPORT_TEXTURE_AS_EMBEDDED. The code below 
	// shows how to change these states.
	IOS_REF.SetBoolProp(EXP_FBX_MATERIAL, true);
	IOS_REF.SetBoolProp(EXP_FBX_TEXTURE, true);
	IOS_REF.SetBoolProp(EXP_FBX_EMBEDDED, pEmbedMedia);
	IOS_REF.SetBoolProp(EXP_FBX_SHAPE, true);
	IOS_REF.SetBoolProp(EXP_FBX_GOBO, true);
	IOS_REF.SetBoolProp(EXP_FBX_ANIMATION, true);
	IOS_REF.SetBoolProp(EXP_FBX_GLOBAL_SETTINGS, true);

	// Export the scene.
	lStatus = lExporter->Export(pScene);

	// Destroy the exporter.
	lExporter->Destroy();

	return lStatus;
}

// to save a scene to a FBX file
bool Export(const char* pFilename, int pFileFormat) {
	return SaveScene(gSdkManager, gScene, pFilename, pFileFormat, true); // true -> embed texture file
}

void AddMaterials(FbxMesh* pMesh)
{
	//// Set material mapping.
	//FbxGeometryElementMaterial* lMaterialElement = pMesh->CreateElementMaterial();
	//lMaterialElement->SetMappingMode(FbxGeometryElement::eByPolygon);
	//lMaterialElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);

	////get the node of mesh, add material for it.
	//FbxNode* lNode = pMesh->GetNode();
	//if (lNode == NULL)
	//	return;
	//lNode->AddMaterial(gMaterial);

	//// We are in eByPolygon, so there's only need for 6 index (a cube has 6 polygons).
	//lMaterialElement->GetIndexArray().SetCount(6);

	//// Set the Index 0 to 6 to the material in position 0 of the direct array.
	//for (int i = 0; i < 6; ++i)
	//	lMaterialElement->GetIndexArray().SetAt(i, 0);

	//==> just use eAllSame, what a stupid example, ignore above --mh
	// Set material mapping.
	FbxGeometryElementMaterial* lMaterialElement = pMesh->CreateElementMaterial();
	lMaterialElement->SetMappingMode(FbxGeometryElement::eAllSame);

	//get the node of mesh, add material for it.
	FbxNode* lNode = pMesh->GetNode();
	if (lNode == NULL)
		return;
	lNode->AddMaterial(gMaterial);
}

// Create a cube mesh. 
FbxNode* CreateCubeMesh(FbxScene* pScene, char* pName)
{
	int i, j;
	FbxMesh* lMesh = FbxMesh::Create(pScene, pName);

	// specify vertex positions
	FbxVector4 lControlPoint0(-50, 0, 50);
	FbxVector4 lControlPoint1(50, 0, 50);
	FbxVector4 lControlPoint2(50, 100, 50);
	FbxVector4 lControlPoint3(-50, 100, 50);
	FbxVector4 lControlPoint4(-50, 0, -50);
	FbxVector4 lControlPoint5(50, 0, -50);
	FbxVector4 lControlPoint6(50, 100, -50);
	FbxVector4 lControlPoint7(-50, 100, -50);

	FbxVector4 lNormalXPos(1, 0, 0);
	FbxVector4 lNormalXNeg(-1, 0, 0);
	FbxVector4 lNormalYPos(0, 1, 0);
	FbxVector4 lNormalYNeg(0, -1, 0);
	FbxVector4 lNormalZPos(0, 0, 1);
	FbxVector4 lNormalZNeg(0, 0, -1);

	//FbxVector4 lNormalXPos(0, 0, 0);
	//FbxVector4 lNormalXNeg(0, 0, 0);
	//FbxVector4 lNormalYPos(0, 0, 0);
	//FbxVector4 lNormalYNeg(0, 0, 0);
	//FbxVector4 lNormalZPos(0, 0, 0);
	//FbxVector4 lNormalZNeg(0, 0, 0);

	// Create control points of tris
	lMesh->InitControlPoints(24);
	FbxVector4* lControlPoints = lMesh->GetControlPoints();

	lControlPoints[0] = lControlPoint0;
	lControlPoints[1] = lControlPoint1;
	lControlPoints[2] = lControlPoint2;
	lControlPoints[3] = lControlPoint3;
	lControlPoints[4] = lControlPoint1;
	lControlPoints[5] = lControlPoint5;
	lControlPoints[6] = lControlPoint6;
	lControlPoints[7] = lControlPoint2;
	lControlPoints[8] = lControlPoint5;
	lControlPoints[9] = lControlPoint4;
	lControlPoints[10] = lControlPoint7;
	lControlPoints[11] = lControlPoint6;
	lControlPoints[12] = lControlPoint4;
	lControlPoints[13] = lControlPoint0;
	lControlPoints[14] = lControlPoint3;
	lControlPoints[15] = lControlPoint7;
	lControlPoints[16] = lControlPoint3;
	lControlPoints[17] = lControlPoint2;
	lControlPoints[18] = lControlPoint6;
	lControlPoints[19] = lControlPoint7;
	lControlPoints[20] = lControlPoint1;
	lControlPoints[21] = lControlPoint0;
	lControlPoints[22] = lControlPoint4;
	lControlPoints[23] = lControlPoint5;

	// We want to have one normal for each vertex (or control point),
	// so we set the mapping mode to eByControlPoint.
	FbxGeometryElementNormal* lGeometryElementNormal = lMesh->CreateElementNormal();

	lGeometryElementNormal->SetMappingMode(FbxGeometryElement::eByControlPoint);

	// Set the normal values for every control point.
	lGeometryElementNormal->SetReferenceMode(FbxGeometryElement::eDirect);

	lGeometryElementNormal->GetDirectArray().Add(lNormalZPos);
	lGeometryElementNormal->GetDirectArray().Add(lNormalZPos);
	lGeometryElementNormal->GetDirectArray().Add(lNormalZPos);
	lGeometryElementNormal->GetDirectArray().Add(lNormalZPos);
	lGeometryElementNormal->GetDirectArray().Add(lNormalXPos);
	lGeometryElementNormal->GetDirectArray().Add(lNormalXPos);
	lGeometryElementNormal->GetDirectArray().Add(lNormalXPos);
	lGeometryElementNormal->GetDirectArray().Add(lNormalXPos);
	lGeometryElementNormal->GetDirectArray().Add(lNormalZNeg);
	lGeometryElementNormal->GetDirectArray().Add(lNormalZNeg);
	lGeometryElementNormal->GetDirectArray().Add(lNormalZNeg);
	lGeometryElementNormal->GetDirectArray().Add(lNormalZNeg);
	lGeometryElementNormal->GetDirectArray().Add(lNormalXNeg);
	lGeometryElementNormal->GetDirectArray().Add(lNormalXNeg);
	lGeometryElementNormal->GetDirectArray().Add(lNormalXNeg);
	lGeometryElementNormal->GetDirectArray().Add(lNormalXNeg);
	lGeometryElementNormal->GetDirectArray().Add(lNormalYPos);
	lGeometryElementNormal->GetDirectArray().Add(lNormalYPos);
	lGeometryElementNormal->GetDirectArray().Add(lNormalYPos);
	lGeometryElementNormal->GetDirectArray().Add(lNormalYPos);
	lGeometryElementNormal->GetDirectArray().Add(lNormalYNeg);
	lGeometryElementNormal->GetDirectArray().Add(lNormalYNeg);
	lGeometryElementNormal->GetDirectArray().Add(lNormalYNeg);
	lGeometryElementNormal->GetDirectArray().Add(lNormalYNeg);


	// Array of polygon vertices.
	int lPolygonVertices[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,12, 13,
		14, 15, 16, 17, 18, 19, 20, 21, 22, 23 };


	// Create UV for Diffuse channel.
	FbxGeometryElementUV* lUVDiffuseElement = lMesh->CreateElementUV("DiffuseUV");
	FBX_ASSERT(lUVDiffuseElement != NULL);

	//// use a reference to prevent access violations? 
	//auto &dir_arr = lUVDiffuseElement->GetDirectArray();
	//auto &index_arr = lUVDiffuseElement->GetIndexArray();

	FbxVector2 lVectors0(0, 0);
	FbxVector2 lVectors1(1, 0);
	FbxVector2 lVectors2(1, 1);
	FbxVector2 lVectors3(0, 1);

	////==> using an index buffer
	//lUVDiffuseElement->SetMappingMode(FbxGeometryElement::eByPolygonVertex);  // eByControlPoint has same effect
	//lUVDiffuseElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);

	//lUVDiffuseElement->GetDirectArray().Add(lVectors0);
	//lUVDiffuseElement->GetDirectArray().Add(lVectors1);
	//lUVDiffuseElement->GetDirectArray().Add(lVectors2);
	//lUVDiffuseElement->GetDirectArray().Add(lVectors3);

	////Now we have set the UVs as eIndexToDirect reference and in eByPolygonVertex  mapping mode
	////we must update the size of the index array.
	//lUVDiffuseElement->GetIndexArray().SetCount(24);

	//==> no index buffer, only vertex buffer
	lUVDiffuseElement->SetMappingMode(FbxGeometryElement::eByControlPoint);
	lUVDiffuseElement->SetReferenceMode(FbxGeometryElement::eDirect);

	std::vector<FbxVector2> uv_coords;
	uv_coords.push_back(lVectors0);
	uv_coords.push_back(lVectors1);
	uv_coords.push_back(lVectors2);
	uv_coords.push_back(lVectors3);

	// Create polygons. Assign texture and texture UV indices.
	for (i = 0; i < 6; i++)  // 6 sides of cube
	{
		// all faces of the cube have the same texture
		lMesh->BeginPolygon(-1, -1, -1, false);

		for (j = 0; j < 4; j++)
		{
			// Control point index
			lMesh->AddPolygon(lPolygonVertices[i * 4 + j]);

			//==> using an index buffer
			//// update the index array of the UVs that map the texture to the face
			//lUVDiffuseElement->GetIndexArray().SetAt(i * 4 + j, j);
			//index_arr.SetAt(i * 4 + j, j); // prevents access violation?

			//==> using only a vertex buffer
			lUVDiffuseElement->GetDirectArray().Add(uv_coords[j]);
			//dir_arr.Add(uv_coords[j]); // prevents access violation?
		}

		lMesh->EndPolygon();
	}

	//// debug
	//FbxLayerElementArrayTemplate<FbxVector2> direct_arr = lUVDiffuseElement->GetDirectArray();
	//int dir_count = direct_arr.GetCount();

	// create a FbxNode
	FbxNode* lNode = FbxNode::Create(pScene, pName);

	// set the node attribute
	lNode->SetNodeAttribute(lMesh);

	// set the shading mode to view texture
	lNode->SetShadingMode(FbxNode::eTextureShading);
	//lNode->SetShadingMode(FbxNode::eFlatShading);

	// rescale the cube
	lNode->LclScaling.Set(FbxVector4(0.3, 0.3, 0.3));

	// add material
	AddMaterials(lMesh);

	// return the FbxNode
	return lNode;
}

void most_basic_cube() {

	// create the scene (creates FbxManager / FbxScene)
	CreateScene();

	// create a FbxNode for our cube
	FbxNode* lCube = CreateCubeMesh(gScene, "cube_name");

	// add cube to scene
	gScene->GetRootNode()->AddChild(lCube);

	// export to .fbx
	//Export("most_basic_cube.fbx", -1);
	Export("C:/Users/maxhu/Desktop/uvatlas_example/fbx_sdk/most_basic_cube.fbx", -1);

}

int main(int argc, char** argv) {
	most_basic_cube();
}
