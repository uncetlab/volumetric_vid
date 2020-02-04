
/* Generates a UV-gradient .bmp file for use as a texture for a mesh with the specified texture coordinates.
 * This is for visualization purposes, you could just use a standard gradient.png file, but this function
 * colors in the unused areas of the texture with black, so it's clear what parts of the texture file are used.
 *
 * input:
 *  - file_name: the name of the output .bmp
 *  - tex_coords: A list of UV coords. Assume every 3 coords makes up a tri. (corresponds to **an element** from pcl::TextureMesh.tex_coordinates)
 */
void generateGradientTexture(
	const std::string &file_name,
	std::vector<Eigen::Vector2f,
	Eigen::aligned_allocator<Eigen::Vector2f>> &tex_coords
);

/* Generates a texture map (.bmp) using the specified uv coordinates
 *
 * input:
 *  - output_file_name
 *  - tex_coords: a list of uv coords (for each img). Assume every 3 make up a tri
 *  - img_coords: a list of image coords (for each img). Assume every 3 make up a tri
 *  - img_files: a list of images files
 */
void generateUVTextureFromImages(
	const std::string &file_name, 
	std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> &tex_coords,
	std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> &img_coords,
	std::vector<std::string> &img_files
);