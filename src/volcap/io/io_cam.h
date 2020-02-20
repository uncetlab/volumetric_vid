#include <pcl/surface/texture_mapping.h>
#include <string>
#include <vector>

//! the volcap namespace
namespace volcap {

	//! the io namespace
	namespace io {

		/**
		 * @brief load CMU's panoptic dataset camera params into PCL's CameraVector
		 * 
		 * @param[in] calibration_file calibration json file
		 * @param[out] cam_list output vector Camera objects are placed into
		 * @param[in] cams_to_use specify a list of camera names to load (cameras not in list are ignored)
		 */
		void loadCameraParams(
			std::string calibration_file,
			pcl::texture_mapping::CameraVector &cam_list,
			std::vector<std::string>* cams_to_use = nullptr
		);

		/**
		 * @brief Manually sets a camera's params (for testing)
		 *
		 * @param[out] cam Camera that gets manually set.
		 */
		void hardcodeLoadCameraParam(pcl::TextureMapping<pcl::PointXYZ>::Camera &cam);
	}
}