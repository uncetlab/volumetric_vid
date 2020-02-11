#include <pcl/TextureMesh.h>
#include <pcl/surface/texture_mapping.h>
#include <blend2d.h>
#include <Eigen/LU>
#include "texturing.h"


void Texturing::generateGradientTexture(const std::string &file_name, std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> &tex_coords) {
	// create image for drawing on
	BLImage img(480, 480, BL_FORMAT_PRGB32);
	BLContext ctx(img);
	ctx.setCompOp(BL_COMP_OP_SRC_OVER);

	// set black background
	ctx.setFillStyle(BLRgba32(0xFF000000));
	ctx.fillAll();

	// create gradients
	ctx.setCompOp(BL_COMP_OP_PLUS); // this should work, except it doesn't bc im guessing the UV mapping overlaps triangles

	BLGradient linear_red(BLLinearGradientValues(0, 0, 0, 480));
	linear_red.addStop(0.0, BLRgba32(0xFF000000));
	linear_red.addStop(1.0, BLRgba32(0xFFFF0000));

	BLGradient linear_green(BLLinearGradientValues(0, 0, 480, 0));
	linear_green.addStop(0.0, BLRgba32(0xFF000000));
	linear_green.addStop(1.0, BLRgba32(0xFF00FF00));

	//ctx.setFillStyle(BLRgba32(0xFFFFFFFF)); // use to color all tris white

	// color in tris
	for (int i = 0; i < tex_coords.size(); i += 3) {  // loop thru faces
		Eigen::Vector2f pt0 = tex_coords[i];
		Eigen::Vector2f pt1 = tex_coords[i + 1];
		Eigen::Vector2f pt2 = tex_coords[i + 2];

		// TODO: scale this better, does Blend2d have a pt struct?
		BLTriangle tri(pt0(0) * 480, (1.0 - pt0(1)) * 480, pt1(0) * 480, (1.0 - pt1(1)) * 480, pt2(0) * 480, (1.0 - pt2(1)) * 480);

		//ctx.setCompOp(BL_COMP_OP_SRC_OVER);
		ctx.setFillStyle(linear_red);
		ctx.fillTriangle(tri);

		//ctx.setCompOp(BL_COMP_OP_PLUS);
		ctx.setFillStyle(linear_green);
		ctx.fillTriangle(tri);
	}

	// write texture map to file
	BLImageCodec codec;
	codec.findByName("BMP");
	img.writeToFile(file_name.c_str(), codec);
}

void Texturing::generateUVTextureFromImages(
	const std::string &file_name,
	std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> &tex_coords,
	std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> &img_coords,
	std::vector<std::string> &img_files
) {
	//==> validate input
	// ASSERT( img_coords.size() == tex_coords.size() == img_files.size() )
	
	//for (int i = 0; i < tex_coords.size(); i++) {
	//	ASSERT(tex_coords[i].size() == img_coords[i].size())
	//}

	int cam_size = tex_coords.size();
	const int BITMAP_SIZE = 512;

	//==> create image for drawing on
	BLImage img(BITMAP_SIZE, BITMAP_SIZE, BL_FORMAT_PRGB32);
	BLContext ctx(img);
	ctx.setCompOp(BL_COMP_OP_SRC_OVER);

	//==> set black background
	ctx.setFillStyle(BLRgba32(0xFF000000));
	ctx.fillAll();

	for (int cam_idx = 0; cam_idx < cam_size; cam_idx++) {
		// get texture
		BLImage texture;
		BLResult err = texture.readFromFile(img_files[cam_idx].c_str());

		// loop thru tris
		for (int vertex_idx = 0; vertex_idx < tex_coords[cam_idx].size(); vertex_idx+=3) {
			Eigen::Vector2f uv_pt0(tex_coords[cam_idx][vertex_idx].x()* BITMAP_SIZE, (1.0 - tex_coords[cam_idx][vertex_idx].y())* BITMAP_SIZE);
			Eigen::Vector2f uv_pt1(tex_coords[cam_idx][vertex_idx+1].x()* BITMAP_SIZE, (1.0 - tex_coords[cam_idx][vertex_idx+1].y())* BITMAP_SIZE);
			Eigen::Vector2f uv_pt2(tex_coords[cam_idx][vertex_idx+2].x()* BITMAP_SIZE, (1.0 - tex_coords[cam_idx][vertex_idx+2].y())* BITMAP_SIZE);


			//Eigen::Vector2f uv_pt1 = tex_coords[cam_idx][vertex_idx + 1] * BITMAP_SIZE;
			//Eigen::Vector2f uv_pt2 = tex_coords[cam_idx][vertex_idx + 2] * BITMAP_SIZE;

			//BLTriangle tri(
			//	uv_pt0(0) * BITMAP_SIZE, (1.0 - uv_pt0(1)) * BITMAP_SIZE,
			//	uv_pt1(0) * BITMAP_SIZE, (1.0 - uv_pt1(1)) * BITMAP_SIZE,
			//	uv_pt2(0) * BITMAP_SIZE, (1.0 - uv_pt2(1)) * BITMAP_SIZE
			//);

			Eigen::Vector2f img_pt0 = img_coords[cam_idx][vertex_idx];
			Eigen::Vector2f img_pt1 = img_coords[cam_idx][vertex_idx + 1];
			Eigen::Vector2f img_pt2 = img_coords[cam_idx][vertex_idx + 2];

			Eigen::Matrix3f img_pts;	//!< starting points
			img_pts <<	img_pt0.x() * 1920, img_pt1.x() * 1920, img_pt2.x() * 1920,
						(1.0 - img_pt0.y()) * 1080, (1.0 - img_pt1.y()) * 1080, (1.0 - img_pt2.y()) * 1080,
						1, 1, 1;

			Eigen::Matrix3f uv_pts;		//!< points after transformation is applied
			uv_pts <<	uv_pt0.x(), uv_pt1.x(), uv_pt2.x(),
						uv_pt0.y(), uv_pt1.y(), uv_pt2.y(),
						1, 1, 1;

			Eigen::Matrix3f inv = img_pts.inverse().eval();
			Eigen::Matrix3f transformation = uv_pts*inv;

			//printf("=====> transformation:\n");
			//std::cout << transformation << "\n";

			//printf("=====> img_pts:\n");
			//std::cout << img_pts << "\n";

			//printf("=====> uv_pts:\n");
			//std::cout << uv_pts << "\n";

			//printf("=====> img_pts*transformation:\n");
			//std::cout << transformation*img_pts << "\n";



			BLPattern pattern(texture);
			BLMatrix2D bl_matrix(
				transformation(0, 0), transformation(1, 0),
				transformation(0, 1), transformation(1, 1),
				transformation(0, 2), transformation(1, 2)
			);
			//pattern.setMatrix(bl_matrix);
			pattern.transform(bl_matrix);
			ctx.setFillStyle(pattern);
			//ctx.fillTriangle(tri);
			ctx.fillTriangle(
				uv_pt0.x(), uv_pt0.y(),
				uv_pt1.x(), uv_pt1.y(),
				uv_pt2.x(), uv_pt2.y()
			);
		}
		
	}

	// write texture map to file
	BLImageCodec codec;
	codec.findByName("BMP");
	img.writeToFile(file_name.c_str(), codec);
	
}

void Texturing::segmentUVMeshByCamera(
	pcl::TextureMesh &mesh,
	pcl::texture_mapping::CameraVector cams,
	std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> &tex_coords,
	std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> &img_coords
) {
	//==> validate input
	// ASSERT(mesh.tex_polygons.size() == 1 && mesh.tex_coordinates.size() == 1);

	//pcl::TextureMapping<pcl::PointXYZ> tm;

	//==> change to PointCloud for easier access
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> transformed_clouds;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_clouds;
	std::vector<std::vector<int>> visible_indices;
	std::vector<std::vector<int>> occluded_indices;
	visible_indices.resize(cams.size());
	occluded_indices.resize(cams.size());

	std::vector<std::vector<bool>> visible_faces;  //!< list of booleans indicating mesh occlusion (per cam)
	//std::vector<std::vector<Eigen::Vector2f>> projected_clouds;
	std::vector<pcl::PointCloud<pcl::PointXY>::Ptr> projected_clouds;
	visible_faces.resize(cams.size());
	//projected_cloud.resize(cams.size());

	//==> create transformed_cloud for each camera (probably suboptimal in terms of memory usage but w/e)
	//==> also compute occlusion for each cloud
	for (int cam_idx = 0; cam_idx < cams.size(); cam_idx++) {
		// transform original cloud in camera coordinates
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		Eigen::Affine3f cam_trans = cams[cam_idx].pose;
		pcl::transformPointCloud(*cloud, *transformed_cloud, cam_trans.inverse());

		transformed_clouds.push_back(transformed_cloud);

		//// compute occlusion per point -- doesnt exactly work
		//pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		//filtered_clouds.push_back(filtered_cloud);
		//tm.removeOccludedPoints(transformed_cloud, filtered_cloud, 1.0, visible_indices[cam_idx], occluded_indices[cam_idx]);

		//==> compute occlusion per face
		pcl::PointCloud<pcl::PointXY>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXY>);
		projected_clouds.push_back(projected_cloud);
		
		// visible_faces - a boolean indicator for whether a face is visible or occluded (for each cam)
		// projected_clouds (a point cloud) - a transformed_cloud projected to cam's 2d plane (use this when building img_coords) (for each cam)
		determineOccludedFaces(mesh, cams[cam_idx], visible_faces[cam_idx], projected_cloud);
	}

	//==> initialize tex_coords / img_coords for each camera
	tex_coords.resize(cams.size());
	img_coords.resize(cams.size());

	//==> loop thru all tris and segment into groups
	const std::vector<pcl::Vertices> &submesh = mesh.tex_polygons[0];
	for (int face_idx = 0; face_idx < submesh.size(); face_idx++) {
		const pcl::Vertices &face = submesh[face_idx];

		// vertices of face
		int point_idx_0 = face.vertices[0];
		int point_idx_1 = face.vertices[1];
		int point_idx_2 = face.vertices[2];

		for (int cam_idx = 0; cam_idx < cams.size(); cam_idx++) {
			std::vector<int> &occlusions = occluded_indices[cam_idx];
			//pcl::PointCloud<pcl::PointXYZ> &transformed_cloud = *transformed_clouds[cam_idx];
			pcl::PointCloud<pcl::PointXY> &projected_cloud = *projected_clouds[cam_idx];

			// skip if face is occluded for this cam
			if (!visible_faces[cam_idx][face_idx]) {
				continue;
			}

			Eigen::Vector2f img_coord0(projected_cloud[point_idx_0].x, projected_cloud[point_idx_0].y);
			Eigen::Vector2f img_coord1(projected_cloud[point_idx_1].x, projected_cloud[point_idx_1].y);
			Eigen::Vector2f img_coord2(projected_cloud[point_idx_2].x, projected_cloud[point_idx_2].y);

			// copy img_coords
			img_coords[cam_idx].push_back(img_coord0);
			img_coords[cam_idx].push_back(img_coord1);
			img_coords[cam_idx].push_back(img_coord2);

			// copy tex_coords
			tex_coords[cam_idx].push_back(mesh.tex_coordinates[0][face_idx * 3]);
			tex_coords[cam_idx].push_back(mesh.tex_coordinates[0][face_idx * 3 + 1]);
			tex_coords[cam_idx].push_back(mesh.tex_coordinates[0][face_idx * 3 + 2]);

			// break for first cam which can see the face (greedy segmentation)
			break;
		}
	}

	////==> loop thru all tris yourself so we can map the texture using uv coords as we go
	//const std::vector<pcl::Vertices> &submesh = mesh.tex_polygons[0];
	//for (int face_idx = 0; face_idx < submesh.size(); face_idx++) {
	//	const pcl::Vertices &face = submesh[face_idx];

	//	// vertices of face
	//	int point_idx_0 = face.vertices[0];
	//	int point_idx_1 = face.vertices[1];
	//	int point_idx_2 = face.vertices[2];

	//	//std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> cam_tex_coords;
	//	//std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> cam_img_coords;

	//	//for (int cam_idx = 0; cam_idx < cams.size(); cam_idx++) {
	//	for (int cam_idx = 3; cam_idx < 4; cam_idx++) {
	//		std::vector<int> &occlusions = occluded_indices[cam_idx];
	//		pcl::PointCloud<pcl::PointXYZ> &transformed_cloud = *transformed_clouds[cam_idx];

	//		// skip if any point of the face is occluded
	//		// TODO: use a set instead of vector for efficiency?
	//		if (std::find(occlusions.begin(), occlusions.end(), point_idx_0) != occlusions.end() ||
	//			std::find(occlusions.begin(), occlusions.end(), point_idx_1) != occlusions.end() ||
	//			std::find(occlusions.begin(), occlusions.end(), point_idx_2) != occlusions.end()) {
	//			continue;
	//		}

	//		// project face to 2d camera plane
	//		Eigen::Vector2f img_coord_0;
	//		Eigen::Vector2f img_coord_1;
	//		Eigen::Vector2f img_coord_2;

	//		tm.getPointUVCoordinates(transformed_cloud[point_idx_0], cams[cam_idx], img_coord_0);
	//		tm.getPointUVCoordinates(transformed_cloud[point_idx_1], cams[cam_idx], img_coord_1);
	//		tm.getPointUVCoordinates(transformed_cloud[point_idx_2], cams[cam_idx], img_coord_2);

	//		Eigen::Vector2f invis_coord(-1.0, -1.0);
	//		if (img_coord_0 == invis_coord || img_coord_1 == invis_coord || img_coord_2 == invis_coord) {
	//			//printf("face not fully visible\n");
	//		} else {  // face is in view
	//			// copy img_coords
	//			img_coords[cam_idx].push_back(img_coord_0);
	//			img_coords[cam_idx].push_back(img_coord_1);
	//			img_coords[cam_idx].push_back(img_coord_2);

	//			// copy tex_coords
	//			tex_coords[cam_idx].push_back(mesh.tex_coordinates[0][face_idx*3]);
	//			tex_coords[cam_idx].push_back(mesh.tex_coordinates[0][face_idx*3 + 1]);
	//			tex_coords[cam_idx].push_back(mesh.tex_coordinates[0][face_idx*3 + 2]);

	//			// break (greedy segmentation)
	//			break;
	//		}
	//	}
	//}
}

void Texturing::determineOccludedFaces(
	const pcl::TextureMesh &mesh, 
	const pcl::TextureMapping<pcl::PointXYZ>::Camera &cam, 
	std::vector<bool> &visible_faces,
	pcl::PointCloud<pcl::PointXY>::Ptr projections  //!< same size as mesh_cloud, indices correspond
) {	
	// initialize

	// transform cloud to camera space
	pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *mesh_cloud);
	pcl::transformPointCloud(*mesh_cloud, *camera_cloud, cam.pose.inverse());

	// project cloud to cam's 2d plane
	//for (int idx_face = 0; idx_face < mesh.tex) {

	//	pcl::PointXY uv_coord1;
	//	pcl::PointXY uv_coord2;
	//	pcl::PointXY uv_coord3;

	//	if (isFaceProjected(cameras[current_cam],
	//		camera_cloud->points[mesh.tex_polygons[0][idx_face].vertices[0]],
	//		camera_cloud->points[mesh.tex_polygons[0][idx_face].vertices[1]],
	//		camera_cloud->points[mesh.tex_polygons[0][idx_face].vertices[2]],
	//		uv_coord1,
	//		uv_coord2,
	//		uv_coord3)
	//	) {
	//		projected_cloud[idx_face]
	//	}
	//}

	// project cloud to cam's 2d plane
	for (int i = 0; i < camera_cloud->size(); i++) {
		Eigen::Vector2f img_coord;
		getPointUVCoordinates((*camera_cloud)[i], cam, img_coord);

		pcl::PointXY pt_xy;
		pt_xy.x = img_coord.x();
		pt_xy.y = img_coord.y();
		//projected_cloud.push_back(img_coord);
		projections->push_back(pt_xy);
	}

	// build projections_face_idx
	std::vector<std::vector<int>> projections_face_idx(mesh_cloud->size());  // list of faces for each point
	//int projections_face_idx[mesh_cloud->size()];
	//std::array<int, mesh_cloud->size()> projections_face_idx;
	const std::vector<pcl::Vertices> &submesh = mesh.tex_polygons[0];
	for (int idx_face = 0; idx_face < submesh.size(); idx_face++) {
		int idx_pt0 = submesh[idx_face].vertices[0];
		int idx_pt1 = submesh[idx_face].vertices[1];
		int idx_pt2 = submesh[idx_face].vertices[2];

		projections_face_idx[idx_pt0].push_back(idx_face);
		projections_face_idx[idx_pt1].push_back(idx_face);
		projections_face_idx[idx_pt2].push_back(idx_face);
	}

	// create kdtree
	pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	kdtree.setInputCloud(projections);
	
	// af first (idx_pcan < current_cam), check if some of the faces attached to previous cameras occlude the current faces
	// then (idx_pcam == current_cam), check for self occlusions. At this stage, we skip faces that were already marked as occluded
	//cpt_invisible = 0;

	// determine occlusion status for all tris
	visible_faces.resize(submesh.size(), true);  // init to the num of faces
	for (int idx_face = 0; idx_face < submesh.size(); idx_face++)  // static_cast<int> (mesh.tex_polygons[idx_pcam].size()
	{
		int idx_pt0 = submesh[idx_face].vertices[0];
		int idx_pt1 = submesh[idx_face].vertices[1];
		int idx_pt2 = submesh[idx_face].vertices[2];

		pcl::PointXYZ pt0 = camera_cloud->points[idx_pt0];
		pcl::PointXYZ pt1 = camera_cloud->points[idx_pt1];
		pcl::PointXYZ pt2 = camera_cloud->points[idx_pt2];

		// project each vertice, if one is out of view, stop
		pcl::PointXY uv_coord1 = (*projections)[idx_pt0];
		pcl::PointXY uv_coord2 = (*projections)[idx_pt1];
		pcl::PointXY uv_coord3 = (*projections)[idx_pt2];

		//pcl::PointXY invis_pt;
		//invis_pt.x = -1.0;
		//invis_pt.y = -1.0;
		if (uv_coord1.x == -1.0 || uv_coord2.x == -1.0 || uv_coord3.x == -1.0) {
			// face is not in the camera's FOV ==> skip!
			continue;
		}

		// get the tri's circumsribed circle
		double radius;
		pcl::PointXY center;
		getTriangleCircumcscribedCircleCentroid(uv_coord1, uv_coord2, uv_coord3, center, radius); // this function yields faster results than getTriangleCircumcenterAndSize

		// get projected points within circumscribed circle
		std::vector<int> idxNeighbors;
		std::vector<float> neighborsSquaredDistance;
		if (kdtree.radiusSearch(center, radius, idxNeighbors, neighborsSquaredDistance) > 0) {

			for (size_t i = 0; i < idxNeighbors.size(); ++i) {
				int idx_neighbor = idxNeighbors[i]; 
				pcl::PointXYZ neighbor = camera_cloud->points[idx_neighbor];

				if (idx_neighbor == idx_pt0 || idx_neighbor == idx_pt1 || idx_neighbor == idx_pt2) {
					// don't count the vertices of our current face
					continue;
				}

				//if (projections_face_idx[idx_neighbor] == -1) {
				//	continue;
				//}

				float max_z_of_face = std::max(camera_cloud->points[idx_pt0].z,
					std::max(camera_cloud->points[idx_pt1].z, camera_cloud->points[idx_pt2].z));

				// if neighbor is farther than all the face's points => it is a candidate for occlusion
				if (max_z_of_face < camera_cloud->points[idx_neighbor].z)  // indexes_uv_to_points[idxNeighbors[i]].idx_cloud
				{
					// check if it falls into the triangle
					if (checkPointInsideTriangle(uv_coord1, uv_coord2, uv_coord3, projections->points[idx_neighbor]))
					{
						// current neighbor is inside triangle and is further away => it's faces are occluded
						//visibility[indexes_uv_to_points[idxNeighbors[i]].idx_face] = false;
						std::vector<int> &neighbors_faces = projections_face_idx[idx_neighbor];
						for (int j = 0; j < projections_face_idx[idx_neighbor].size(); j++) {
							visible_faces[projections_face_idx[idx_neighbor][j]] = false;
						}
					}
				}

				//// if neighbor is closer than all the face's points => it may be occluding our face
				//if (camera_cloud->points[idx_neighbor].z < max_z_of_face)  // indexes_uv_to_points[idxNeighbors[i]].idx_cloud
				//{
				//	// check if it falls into the triangle
				//	if (checkPointInsideTriangle(uv_coord1, uv_coord2, uv_coord3, projections->points[idx_neighbor]))
				//	{
				//		// current neighbor is inside the triangle and is closer => idx_face is occluded -- mh
				//		visible_faces[idx_face] = false;
				//	}
				//}
			}
		}
	}
	
}