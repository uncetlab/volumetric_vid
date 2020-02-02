#include <pcl/TextureMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/conversions.h>
#include <fbxsdk.h>

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
	lStatus = lExporter->Export(pScene);	// dont break on exception 0xc0000005

	// Destroy the exporter.
	lExporter->Destroy();

	return lStatus;
}

// to save a scene to a FBX file. local version
bool Export(const char* pFilename, int pFileFormat, FbxManager* pManager, FbxScene* pScene) {
	return SaveScene(pManager, pScene, pFilename, pFileFormat, true); // true -> embed texture file
}

// add materials to a mesh. local version
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

// convert a pcl::TextureMesh to an FbxMesh, add to pScene
FbxNode* createMesh(pcl::TextureMesh &mesh, FbxScene* pScene, char* pName, FbxSurfacePhong* pMaterial) {
	
	FbxMesh* lMesh = FbxMesh::Create(pScene, pName);

	//==> specify vertex positions and normals

	// convert PCLPointCloud2 to PointCloud
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	std::vector<FbxVector4> vertices;
	std::vector<FbxVector4> normals;
	for (int i = 0; i < cloud->size(); i++) {
		pcl::PointNormal &pcl_v = cloud->points[i];
		FbxVector4 v(pcl_v.x, pcl_v.y, pcl_v.z);
		FbxVector4 normal(pcl_v.normal_x, pcl_v.normal_y, pcl_v.normal_z);
		vertices.push_back(v);
		normals.push_back(normal);
	}

	//==> init control points (a vertex has as many control points as the number of tris it is a part of)
	std::vector<pcl::Vertices> &submesh = mesh.tex_polygons[0];
	int num_tris = submesh.size();
	int num_control_pts = num_tris * 3;

	lMesh->InitControlPoints(num_control_pts);
	FbxVector4* lControlPoints = lMesh->GetControlPoints();

	//==> set normal mapping / reference modes (TODO: generate real normals per face or vertex to use)
	FbxGeometryElementNormal* lGeometryElementNormal = lMesh->CreateElementNormal();
	//lGeometryElementNormal->SetMappingMode(FbxGeometryElement::eByControlPoint);
	lGeometryElementNormal->SetMappingMode(FbxGeometryElement::eAllSame); // quick hack, use a fake normal for all control points
	lGeometryElementNormal->SetReferenceMode(FbxGeometryElement::eDirect);

	FbxVector4 lNormalXPos(1, 0, 0);  // fake normal
	lGeometryElementNormal->GetDirectArray().Add(lNormalXPos);

	//==> set UV mapping / reference modes
	FbxGeometryElementUV* lUVDiffuseElement = lMesh->CreateElementUV("DiffuseUV");
	FBX_ASSERT(lUVDiffuseElement != NULL);

	// use a reference to prevent access violation errors?
	auto &dir_arr = lUVDiffuseElement->GetDirectArray();  // try using this instead of lUVDiffuseElement->GetDirectArray() in loop if error occurs
	//auto &index_arr = lUVDiffuseElement->GetIndexArray();

	////==> using an index buffer + vertex buffer (TODO: if UVAtlas output a more reasonable UV map, where faces shared vertices, we should use an index buffer)
	//lUVDiffuseElement->SetMappingMode(FbxGeometryElement::eByControlPoint); // eByPolygonVertex // eByControlPoint has same effect
	//lUVDiffuseElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);

	////Now we have set the UVs as eIndexToDirect reference and in eByPolygonVertex mapping mode
	////we must update the size of the index array.
	//lUVDiffuseElement->GetIndexArray().SetCount(num_tris * 3);

	//==> using a vertex buffer (no index buffer)
	lUVDiffuseElement->SetMappingMode(FbxGeometryElement::eByControlPoint); // eByPolygonVertex would have same effect here
	lUVDiffuseElement->SetReferenceMode(FbxGeometryElement::eDirect);

	for (int tri_idx = 0; tri_idx < submesh.size(); tri_idx++) {
		lMesh->BeginPolygon(-1, -1, -1, false);

		for (int i = 0; i < 3; i++) {
			//==> set control points and their normals
			lControlPoints[tri_idx*3 + i] = vertices[submesh[tri_idx].vertices[i]];
			//lGeometryElementNormal->GetDirectArray().Add(normals[submesh[tri_idx].vertices[i]]);  // (TODO: for when we use mapping mode eByControlPoint)

			//==> add a control point to the current polygon
			lMesh->AddPolygon(tri_idx * 3 + i);

			//==> UV mapping: using an index buffer
			// update the index array of the UVs that map the texture to the face
			//lUVDiffuseElement->GetIndexArray().SetAt(tri_idx * 3 + i, i);
			//lUVDiffuseElement->GetIndexArray().Add(i);
			//index_arr.Add(i);

			//==> UV mapping: using only vertex buffer
			Eigen::Vector2f &eigen_vec = mesh.tex_coordinates[0][tri_idx * 3 + i];
			FbxVector2 uv(eigen_vec(0), eigen_vec(1));
			lUVDiffuseElement->GetDirectArray().Add(uv);
		}

		lMesh->EndPolygon();
	}

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
	AddMaterials(lMesh, pMaterial);

	// return the FbxNode
	return lNode;
}

/* converts pcl::TextureMesh to fbx, exports to .fbx file
 * assumes the TextureMesh has 1 of each tex_polygons, tex_coordinates, tex_materials
 *
 */
bool fbxFromTextureMesh(pcl::TextureMesh &mesh, const std::string mesh_name, std::string output_full_path) {

	//==> declare local fbx variables
	FbxManager* lManager = NULL;
	FbxScene* lScene = NULL;
	FbxFileTexture* lTexture = NULL;
	FbxSurfacePhong* lMaterial = NULL;

	//==> initialize fbx variables
	if (InitializeSdkObjects(lManager, lScene) == false)
	{
		return false;
	}

	// create a single texture shared by all meshes
	CreateTexture(lTexture, lScene, mesh.tex_materials[0].tex_file);

	// create a material shared by all faces of all cubes, containing the provided texture
	CreateMaterial(lMaterial, lTexture, lScene);

	//==> create a FbxNode for our mesh
	FbxNode* lMesh = createMesh(mesh, lScene, "mesh_name", lMaterial);

	//==> add mesh to scene
	lScene->GetRootNode()->AddChild(lMesh);

	//==> export to .fbx
	Export(output_full_path.c_str(), -1, lManager, lScene);

	return true;
}

/* loads all TextureMeshes (saved as .obj files) from a dir (assumes every file is a .obj file)
 * works with .obj files which only have 1 material
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
		std::string filename = path.stem().string();  // .filename() includes extension, use .stem() instead

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

		//edit texture files to be full paths (necessary when creating textures)
		mesh->tex_materials[0].tex_file = dir_name + "/" + mesh->tex_materials[0].tex_file;

		meshes.push_back(mesh);
		mesh_filenames.push_back(filename);
	}
}

int main(int argc, char** argv) {

	////==> single example
	//pcl::TextureMesh tmesh;
	//pcl::io::loadOBJFile("C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/ptcloud_hd00000380_normals_cleaned.obj", tmesh);
	//fbxFromTextureMesh(tmesh);

	//==> convert a dir of .obj files into .fbx
	std::vector<pcl::TextureMeshPtr> meshes;
	std::vector<std::string> mesh_filenames;
	std::string input_dir = "C:/Users/maxhu/etlab/volumetric_capture/panoptic-toolbox/171026_pose3/kinoptic_ptclouds/textured_mesh/uvatlas_gradient";
	load_meshes_from_dir(input_dir, meshes, mesh_filenames);

	std::string output_dir = input_dir + "/fbx/";

	for (int i = 0; i < meshes.size(); i++) {
		fbxFromTextureMesh(*meshes[i], mesh_filenames[i], output_dir + mesh_filenames[i]);
	}
}
