#include <pcl/TextureMesh.h>
#include <fbxsdk.h>

//! the volcap namespace
namespace volcap {

	//! the io namespace
	namespace io {

		bool InitializeSdkObjects(FbxManager*& pManager, FbxScene*& pScene);

		// local version of create texture
		void CreateTexture(FbxFileTexture*& pTexture, FbxScene* pScene, std::string tex_file);

		// local version of create material
		void CreateMaterial(FbxSurfacePhong*& pMaterial, FbxFileTexture* pTexture, FbxScene* pScene);

		// to save a scene to a FBX file
		bool SaveScene(FbxManager* pSdkManager, FbxDocument* pScene, const char* pFilename, int pFileFormat, bool pEmbedMedia);

		// to save a scene to a FBX file. local version
		bool Export(const char* pFilename, int pFileFormat, FbxManager* pManager, FbxScene* pScene);

		// add materials to a mesh. local version
		void AddMaterials(FbxMesh* pMesh, FbxSurfacePhong* pMaterial);

		// convert a pcl::TextureMesh to an FbxMesh, add to pScene
		FbxNode* createMesh(pcl::TextureMesh &mesh, FbxScene* pScene, char* pName, FbxSurfacePhong* pMaterial);

		/**
		 * @brief converts pcl::TextureMesh to fbx, exports to .fbx file
		 * @remark assumes the TextureMesh has 1 of each tex_polygons, tex_coordinates, tex_materials
		 */
		bool fbxFromTextureMesh(pcl::TextureMesh &mesh, const std::string mesh_name, std::string output_full_path);
		//bool fbxFromTextureMesh(pcl::TextureMesh &mesh, char* mesh_name, std::string output_full_path);


		/* TODO: move above functions into an FbxConverter class 
		class FbxConverter {
		 public:
		}
		*/
	}
}