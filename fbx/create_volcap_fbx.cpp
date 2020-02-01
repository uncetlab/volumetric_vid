#include <pcl/TextureMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/conversions.h>
#include <fbxsdk.h>
//#include <vector>

// declare globals
//FbxManager*   gSdkManager = NULL;
//FbxScene*        gScene = NULL;
//FbxFileTexture*  gTexture = NULL;
//FbxSurfacePhong* gMaterial = NULL;

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

//// Create a global texture for cube.
//void CreateTexture(FbxScene* pScene, std::string tex_file)
//{
//	gTexture = FbxFileTexture::Create(pScene, "Diffuse Texture");
//
//	FbxString lTexPath = tex_file.c_str();  // this texture gets embedded in the binary .fbx file
//
//	// Set texture properties.
//	gTexture->SetFileName(lTexPath.Buffer());  // must pass in an absolute path
//	gTexture->SetTextureUse(FbxTexture::eStandard);
//	gTexture->SetMappingType(FbxTexture::eUV);
//	gTexture->SetMaterialUse(FbxFileTexture::eModelMaterial);
//	gTexture->SetSwapUV(false);
//	gTexture->SetTranslation(0.0, 0.0);
//	gTexture->SetScale(1.0, 1.0);
//	gTexture->SetRotation(0.0, 0.0);
//}
//
//// Create global material for cube.
//void CreateMaterial(FbxScene* pScene)
//{
//	FbxString lMaterialName = "material";
//	FbxString lShadingName = "Phong";
//	FbxDouble3 lBlack(0.0, 0.0, 0.0);
//	FbxDouble3 lRed(1.0, 0.0, 0.0);
//	FbxDouble3 lDiffuseColor(0.75, 0.75, 0.0);
//	gMaterial = FbxSurfacePhong::Create(pScene, lMaterialName.Buffer());
//
//	// Generate primary and secondary colors.
//	gMaterial->Emissive.Set(lBlack);
//	gMaterial->Ambient.Set(lRed);
//	gMaterial->Diffuse.Set(lDiffuseColor);
//	gMaterial->TransparencyFactor.Set(40.5);
//	gMaterial->ShadingModel.Set(lShadingName);
//	gMaterial->Shininess.Set(0.5);
//
//	// the texture need to be connected to the material on the corresponding property
//	if (gTexture)
//		gMaterial->Diffuse.ConnectSrcObject(gTexture);
//}

// local version of create texture
void CreateTexture(FbxFileTexture*& pTexture, FbxScene* pScene, std::string tex_file)
{
	pTexture = FbxFileTexture::Create(pScene, "Diffuse Texture");

	FbxString lTexPath = tex_file.c_str();  // this texture gets embedded in the binary .fbx file

	// Set texture properties.
	bool success = pTexture->SetFileName(lTexPath.Buffer());  // must pass in an absolute path
	pTexture->SetTextureUse(FbxTexture::eStandard);
	pTexture->SetMappingType(FbxTexture::eUV);
	pTexture->SetMaterialUse(FbxFileTexture::eModelMaterial);
	pTexture->SetSwapUV(false);
	pTexture->SetTranslation(0.0, 0.0);
	pTexture->SetScale(1.0, 1.0);
	pTexture->SetRotation(0.0, 0.0);
}

// local version of create material
void CreateMaterial(FbxSurfacePhong*& pMaterial, FbxFileTexture* pTexture, FbxScene* pScene)
{
	FbxString lMaterialName = "material";
	FbxString lShadingName = "Phong";
	FbxDouble3 lBlack(0.0, 0.0, 0.0);
	FbxDouble3 lRed(1.0, 0.0, 0.0);
	FbxDouble3 lDiffuseColor(0.75, 0.75, 0.0);
	pMaterial = FbxSurfacePhong::Create(pScene, lMaterialName.Buffer());

	// Generate primary and secondary colors.
	pMaterial->Emissive.Set(lBlack);
	pMaterial->Ambient.Set(lRed);
	pMaterial->Diffuse.Set(lDiffuseColor);
	pMaterial->TransparencyFactor.Set(40.5);
	pMaterial->ShadingModel.Set(lShadingName);
	pMaterial->Shininess.Set(0.5);

	// the texture need to be connected to the material on the corresponding property
	if (pTexture)
		pMaterial->Diffuse.ConnectSrcObject(pTexture);
}

//// to create a basic scene
//bool CreateScene()
//{
//	// Initialize the FbxManager and the FbxScene
//	if (InitializeSdkObjects(gSdkManager, gScene) == false)
//	{
//		return false;
//	}
//
//	// create a single texture shared by all cubes
//	CreateTexture(gScene);
//
//	// create a material shared by all faces of all cubes
//	CreateMaterial(gScene);
//
//	return true;
//}

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

	FbxStatus status_before = lExporter->GetStatus();
	lStatus = lExporter->Export(pScene);	// dont break on exception 0xc0000005
	//lStatus = lExporter->Export(pScene, true);	// dont break on exception 0xc0000005 // access violation happens even on new thread
	FbxStatus status_after = lExporter->GetStatus();

	// Destroy the exporter.
	lExporter->Destroy();

	return lStatus;
}

//// to save a scene to a FBX file
//bool Export(const char* pFilename, int pFileFormat) {
//	return SaveScene(gSdkManager, gScene, pFilename, pFileFormat, true); // true -> embed texture file
//}

// to save a scene to a FBX file
bool Export(const char* pFilename, int pFileFormat, FbxManager* pManager, FbxScene* pScene) {
	return SaveScene(pManager, pScene, pFilename, pFileFormat, true); // true -> embed texture file
}

//void AddMaterials(FbxMesh* pMesh)
//{
//	// Set material mapping.
//	FbxGeometryElementMaterial* lMaterialElement = pMesh->CreateElementMaterial();
//	lMaterialElement->SetMappingMode(FbxGeometryElement::eAllSame);
//
//	//get the node of mesh, add material for it.
//	FbxNode* lNode = pMesh->GetNode();
//	if (lNode == NULL)
//		return;
//	lNode->AddMaterial(gMaterial);
//}

// local version of AddMaterials
void AddMaterials(FbxMesh* pMesh, FbxSurfacePhong* pMaterial)
{
	// Set material mapping.
	FbxGeometryElementMaterial* lMaterialElement = pMesh->CreateElementMaterial();
	lMaterialElement->SetMappingMode(FbxGeometryElement::eAllSame);

	//get the node of mesh, add material for it.
	FbxNode* lNode = pMesh->GetNode();
	if (lNode == NULL)
		return;
	lNode->AddMaterial(pMaterial);
}

FbxNode* createMesh(pcl::TextureMesh &mesh, FbxScene* pScene, char* pName, FbxSurfacePhong* pMaterial) {
	int i, j;
	FbxMesh* lMesh = FbxMesh::Create(pScene, pName);


	//==> specify vertex positions and normals
	// convert PCLPointCloud2 to PointCloud
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	//FbxVector4 lNormalXPos(1, 0, 0);

	//FbxVector4 vertices[mesh.cloud.width()];
	std::vector<FbxVector4> vertices;
	std::vector<FbxVector4> normals;
	for (int i = 0; i < cloud->size(); i++) {
		pcl::PointNormal &pcl_v = cloud->points[i];
		FbxVector4 v(pcl_v.x, pcl_v.y, pcl_v.z);
		FbxVector4 normal(pcl_v.normal_x, pcl_v.normal_y, pcl_v.normal_z);
		vertices.push_back(v);
		normals.push_back(normal);
	}

	//==> Create control points of tris and set normals (a vertex has as many control points as the number of tris it is a part of)
	// currently our obj files have all normals as 0
	std::vector<pcl::Vertices> &submesh = mesh.tex_polygons[0];
	int num_tris = submesh.size();
	int num_control_pts = num_tris * 3;

	lMesh->InitControlPoints(num_control_pts);
	FbxVector4* lControlPoints = lMesh->GetControlPoints();

	// normal element
	FbxGeometryElementNormal* lGeometryElementNormal = lMesh->CreateElementNormal();
	//lGeometryElementNormal->SetMappingMode(FbxGeometryElement::eByControlPoint);
	lGeometryElementNormal->SetMappingMode(FbxGeometryElement::eAllSame); // quick hack
	lGeometryElementNormal->SetReferenceMode(FbxGeometryElement::eDirect);

	FbxVector4 lNormalXPos(1, 0, 0);
	lGeometryElementNormal->GetDirectArray().Add(lNormalXPos);

	// UV diffuse element
	FbxGeometryElementUV* lUVDiffuseElement = lMesh->CreateElementUV("DiffuseUV");
	FBX_ASSERT(lUVDiffuseElement != NULL);

	// use a reference to prevent access violations? 
	auto &dir_arr = lUVDiffuseElement->GetDirectArray();
	auto &index_arr = lUVDiffuseElement->GetIndexArray();

	FbxVector2 lVectors0(0, 0);
	FbxVector2 lVectors1(1, 0);
	FbxVector2 lVectors2(1, 1);

	////==> using an index buffer
	//lUVDiffuseElement->SetMappingMode(FbxGeometryElement::eByControlPoint); // eByPolygonVertex // eByControlPoint has same effect
	//lUVDiffuseElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);

	////lUVDiffuseElement->GetDirectArray().Add(lVectors0);
	////lUVDiffuseElement->GetDirectArray().Add(lVectors1);
	////lUVDiffuseElement->GetDirectArray().Add(lVectors2);

	//dir_arr.Add(lVectors0);
	//dir_arr.Add(lVectors1);
	//dir_arr.Add(lVectors2);

	////Now we have set the UVs as eIndexToDirect reference and in eByPolygonVertex  mapping mode
	////we must update the size of the index array.
	//lUVDiffuseElement->GetIndexArray().SetCount(num_tris * 3);

	//==> no index buffer, only vertex buffer (access violation error)
	lUVDiffuseElement->SetMappingMode(FbxGeometryElement::eByControlPoint); // eByPolygonVertex would have same effect here
	lUVDiffuseElement->SetReferenceMode(FbxGeometryElement::eDirect); // eIndexToDirect

	//std::vector<FbxVector2> uv_coords;
	//uv_coords.push_back(lVectors0);
	//uv_coords.push_back(lVectors1);
	//uv_coords.push_back(lVectors2);

	for (int tri_idx = 0; tri_idx < submesh.size(); tri_idx++) {
		lMesh->BeginPolygon(-1, -1, -1, false);

		for (int i = 0; i < 3; i++) {
			// set control points and their normals
			lControlPoints[tri_idx*3 + i] = vertices[submesh[tri_idx].vertices[i]];
			//lGeometryElementNormal->GetDirectArray().Add(normals[submesh[tri_idx].vertices[i]]);  // currently all the imported normals are 0, so don't use this

			lMesh->AddPolygon(tri_idx * 3 + i);

			//==> using an index buffer
			// update the index array of the UVs that map the texture to the face
			//lUVDiffuseElement->GetIndexArray().SetAt(tri_idx * 3 + i, i);
			//lUVDiffuseElement->GetIndexArray().Add(i);
			//index_arr.Add(i);

			//==> using only vertex buffer
			Eigen::Vector2f &eigen_vec = mesh.tex_coordinates[0][tri_idx * 3 + i];
			FbxVector2 uv(eigen_vec(0), eigen_vec(1));
			lUVDiffuseElement->GetDirectArray().Add(uv);

			//lUVDiffuseElement->GetDirectArray().Add(uv_coords[i]);
			//dir_arr.Add(uv_coords[i]);
		}

		lMesh->EndPolygon();

	}

	////// debug
	//FbxLayerElementArrayTemplate<FbxVector2> &direct_arr_first = lUVDiffuseElement->GetDirectArray();
	//FbxLayerElementArrayTemplate<int> &index_arr_first = lUVDiffuseElement->GetIndexArray();

	//FbxLayerElementArrayTemplate<FbxVector2> &direct_arr_second = lUVDiffuseElement->GetDirectArray();
	//FbxLayerElementArrayTemplate<int> &index_arr_second = lUVDiffuseElement->GetIndexArray();
	//int dir_count = dir_arr.GetCount();
	//int index_count = index_arr.GetCount();

	// create a FbxNode
	FbxNode* lNode = FbxNode::Create(pScene, pName);

	// set the node attribute
	lNode->SetNodeAttribute(lMesh);

	// set the shading mode to view texture
	lNode->SetShadingMode(FbxNode::eTextureShading);
	//lNode->SetShadingMode(FbxNode::eFlatShading);

	// rescale
	//lNode->LclScaling.Set(FbxVector4(0.3, 0.3, 0.3));

	// add material
	//AddMaterials(lMesh);
	AddMaterials(lMesh, pMaterial);

	// return the FbxNode
	return lNode;
}

/* exports pcl::TextureMesh to fbx
 * assumes the TextureMesh has 1 of each tex_polygons, tex_coordinates, tex_materials
 *
 */
bool fbxFromTextureMesh(pcl::TextureMesh &mesh, const std::string mesh_name, std::string output_full_path) {

	//==> create the scene (creates FbxManager / FbxScene)
	//CreateScene();

	// Initialize the FbxManager and the FbxScene
	//if (InitializeSdkObjects(gSdkManager, gScene) == false)
	//{
	//	return false;
	//}

	// try having locals instead of globals
	FbxManager* lManager = NULL;
	FbxScene* lScene = NULL;
	FbxFileTexture* lTexture = NULL;
	FbxSurfacePhong* lMaterial = NULL;
	if (InitializeSdkObjects(lManager, lScene) == false)
	{
		return false;
	}

	// create a single texture shared by all cubes
	//CreateTexture(gScene, mesh.tex_materials[0].tex_file);
	CreateTexture(lTexture, lScene, mesh.tex_materials[0].tex_file);
	//CreateTexture(lTexture, lScene, "C:/Users/maxhu/Desktop/uvatlas_example/fbx_sdk/Crate.jpg");
	//CreateTexture(lTexture, lScene, "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient/ptcloud_hd00000381_normals_cleaned.bmp");
	//CreateTexture(gScene, "C:/Users/maxhu/Desktop/uvatlas_example/fbx_sdk/Crate.jpg");

	// create a material shared by all faces of all cubes
	//CreateMaterial(gScene);
	CreateMaterial(lMaterial, lTexture, lScene);

	//==> create a FbxNode for our mesh
	//FbxNode* lMesh = createMesh(mesh, gScene, "mesh_name");
	FbxNode* lMesh = createMesh(mesh, lScene, "mesh_name", lMaterial);
	//FbxNode* lMesh = createMesh(mesh, gScene, mesh_name.c_str());

	//==> add cube to scene
	//gScene->GetRootNode()->AddChild(lMesh);
	lScene->GetRootNode()->AddChild(lMesh);

	//==> export to .fbx
	//Export("C:/Users/maxhu/Desktop/uvatlas_example/fbx_sdk/test_panoptic.fbx", -1);
	//Export("C:/Users/maxhu/Desktop/uvatlas_example/fbx_sdk/test_ptcloud_hd00000380_normals_cleaned.fbx", -1);
	//Export(output_full_path.c_str(), -1);
	Export(output_full_path.c_str(), -1, lManager, lScene);

	return true;
}

/* loads all TextureMeshes (saved as .obj files) from a dir (assumes every file is a .obj file)
 *
 */
void load_meshes_from_dir(const std::string dir_name, std::vector<pcl::TextureMeshPtr> &meshes, std::vector<std::string> &mesh_filenames) {
	boost::filesystem::path input_dir(dir_name);

	boost::filesystem::directory_iterator it{ input_dir };
	int i = 0;
	while (it != boost::filesystem::directory_iterator{}) {
		// Get input / output paths
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path path = entry.path();
		//std::string filename = path.filename().string();
		std::string filename = path.stem().string();  // .filename() includes extension

		if (boost::filesystem::extension(path) != ".obj") {
			printf("skipping file with extension: %s\n", boost::filesystem::extension(path).c_str());
			continue;
		}
		printf("loading TextureMesh %i\n", i++);

		std::string mesh_path = path.string();

		pcl::TextureMeshPtr mesh(boost::make_shared<pcl::TextureMesh>());
		pcl::io::loadOBJFile(mesh_path, *mesh);		// this is broken for TextureMeshes with multiple materials, 
													// all texture coordinates get loaded into the first submesh 
													// (any other submeshes get no texture coordinates)

		//edit texture files to be full paths
		mesh->tex_materials[0].tex_file = dir_name + "/" + mesh->tex_materials[0].tex_file;

		//// quick hack  -- this just uses material_0 for every submesh, since visualization only supports 1 material
		//// (this causes each submesh other than the first to look wrong)
		//pcl::io::loadPolygonFileOBJ(mesh_path, *mesh);
		//pcl::TextureMesh mesh2;
		//pcl::io::loadOBJFile(mesh_path, mesh2);
		//mesh->tex_materials.clear();
		//mesh->tex_materials.push_back(mesh2.tex_materials[0]);

		meshes.push_back(mesh);
		//mesh_filenames.push_back(mesh_path);
		mesh_filenames.push_back(filename);
	}
}

int main(int argc, char** argv) {

	//// load in TextureMesh
	//pcl::TextureMesh tmesh;
	////pcl::io::loadOBJFile("C:/Users/maxhu/Desktop/uvatlas_example/test_panoptic.obj", tmesh);
	//pcl::io::loadOBJFile("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/ptcloud_hd00000380_normals_cleaned.obj", tmesh);

	//fbxFromTextureMesh(tmesh);

	std::vector<pcl::TextureMeshPtr> meshes;
	std::vector<std::string> mesh_filenames;
	std::string input_dir = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient";
	load_meshes_from_dir(input_dir, meshes, mesh_filenames);

	std::string output_dir = input_dir + "/fbx/";

	for (int i = 0; i < meshes.size(); i++) {
		fbxFromTextureMesh(*meshes[i], mesh_filenames[i], output_dir + mesh_filenames[i]);
	}
}
