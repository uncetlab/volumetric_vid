
/* Generates a UV-gradient .png file for use as a texture for a mesh with the specified texture coordinates.
 * This is for visualization purposes, you could just use a standard gradient.png file, but this function
 * colors in the unused areas of the texture with black, so it's clear what parts of the texture file are used.
 *
 * input:
 *  - texture_coordinates
 */
void generateGradientTexture(const std::string &file_name, std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> &tex_coords);

void generateUVTextureFromImages();