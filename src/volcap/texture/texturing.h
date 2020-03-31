#include <pcl/surface/texture_mapping.h>
#include <vector>

//! the volcap namespace
namespace volcap {

	//! the texture namespace
	namespace texture {

		//template<typename PointInT>
		class Texturing : pcl::TextureMapping<pcl::PointXYZ> {
		 public:

			/**
			 * @brief Generates a UV-gradient .bmp file for use as a texture for a mesh with the specified texture coordinates.
			 *
			 * @remark This is for visualization purposes, you could just use a standard gradient.png file, but this function
			 *		colors in the unused areas of the texture with black, so it's clear what parts of the texture file are used.
			 *
			 * @param[in] file_name		the name of the output .bmp
			 * @param[in] tex_coords	A list of UV coords. Assume every 3 coords makes up a tri. (corresponds to **an element** from pcl::TextureMesh.tex_coordinates)
			 */
			void generateGradientTexture(
				const std::string &file_name,
				std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> &tex_coords
			);

			/**
			 * @brief Generates a texture map (.bmp) using the specified uv coordinates
			 * @remark tex_coods, img_coords, tri_verts all have the same dimension, and the groups
			 *  of 3 within each param should correspond to the same tri
			 *
			 * @param[in] file_name
			 * @param[in] tex_coords	a list of uv coords (for each img). Assume every 3 make up a tri
			 * @param[in] img_coords	a list of image coords (for each img). Assume every 3 make up a tri
			 * @param[in] img_files		a list of images files
			 * @param[in] tri_verts		a list of vertex indices (for each img). Assume every 3 make up a tri
			 * @param[in] cam_weights	a list of cam weights (for each img). Only 1 value per tri => Size of inner
			 *  vec is 1/3 of `tex_coords`, `img_coords` & `tri_verts`
			 */
			void generateUVTextureFromImages(
				const std::string &file_name,
				std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> &tex_coords,
				std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> &img_coords,
				std::vector<std::string> &img_files,
				std::vector<std::vector<int>> &tri_verts,
				std::vector<std::vector<float>> &cam_weights
			);

			/**
			 * @brief Segments the mesh into groups by camera visibility status
			 *
			 * @remark Assumes mesh.tex_polygons.size() and mesh.tex_coordinates.size() are both 1
			 * @remark Assumes mesh.tex_coordinates contains desired uv mapping for generated texture (generated by UVAtlas)
			 * @remark Output can be used as input to generateUVTextureFromImages()
			 *
			 * @param[in] mesh			mesh which needs segmenting
			 * @param[in] cams			cameras by which to segment
			 * @param[out] tex_coords	a list of uv coords (for each img). Assume every 3 make up a tri
			 * @param[out] img_coords	a list of image coords (for each img). Assume every 3 make up a tri
			 * @param[out] tri_verts	a list of vertex indices (for each img). Assume every 3 make up a tri
			 * @param[out] cam_weights	a list of cam weights (for each img). Only 1 value per tri => Size of inner
			 *  vec is 1/3 of `tex_coords`, `img_coords` & `tri_verts`
			 */
			void segmentUVMeshByCamera(
				pcl::TextureMesh &mesh,
				pcl::texture_mapping::CameraVector cams,
				std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> &tex_coords,
				std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> &img_coords,
				std::vector<std::vector<int>> &tri_verts,
				std::vector<std::vector<float>> &cam_weights
			);

			/**
			 * @brief Uses a kdtree to determine occluded faces as in pcl::TextureMapping::textureMeshwithMultipleCameras
			 *
			 * @param[in] camera_cloud		a point cloud transformed to camera's space
			 * @param[in] projected_cloud	camera_cloud projected to 2d cam surface
			 */
			 //void determineOccludedFaces(
			 //	pcl::PointCloud<PointInT> &camera_cloud,
			 //	tex_polygons
			 //);

			 /**
			  * @brief Uses a kdtree to determine occluded faces (from `cam`'s perspective)
			  *
			  * @remark algorithm adapted from pcl::TextureMapping::textureMeshwithMultipleCameras()
			  * @todo Perform back-face culling to easily remove faces not facing the camera. This should remove many faces from the start and speed things up.
			  *
			  * @param[in] mesh				the mesh whose faces we check for occlusion
			  * @param[in] cam				the camera whose perspective we view from
			  * @param[out] visible_faces	list of booleans indicating face occlusion (per cam)
			  * @param[out] projections		clouds projected to 2d cam planes (per cam)
			  */
			void determineOccludedFaces(
				const pcl::TextureMesh &mesh,
				const pcl::TextureMapping<pcl::PointXYZ>::Camera &cam,
				std::vector<bool> &visible_faces,
				pcl::PointCloud<pcl::PointXY>::Ptr projections
			);
		};
	}
}

