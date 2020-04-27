#include <pcl/io/ply_io.h>
#include <boost/filesystem.hpp>

template <typename PointT>
void volcap::io::load_clouds_from_dir(
	const std::string dir_name,
	std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
	std::vector<std::string> &cloud_filenames,
	int meshes_to_load
) {
	boost::filesystem::path input_dir(dir_name);

	boost::filesystem::directory_iterator it{ input_dir };
	int i = 0;
	while (i < meshes_to_load && it != boost::filesystem::directory_iterator{}) {
		// Get input / output paths
		boost::filesystem::directory_entry entry = *it++;
		boost::filesystem::path path = entry.path();

		// skip if entry is a directory
		if (!boost::filesystem::is_regular_file(path))
			continue;

		printf("loading cloud %i\n", i++);

		//std::string cloud_id = path.string();

		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::io::loadPLYFile(path.string(), *cloud);

		clouds.push_back(cloud);

		std::string filename = path.stem().string();  // .filename() includes extension, use .stem() instead
		cloud_filenames.push_back(filename);
	}
}
