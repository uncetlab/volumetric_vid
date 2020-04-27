#ifndef IO_CAM_H
#define IO_CAM_H

#include <pcl/surface/texture_mapping.h>
#include <string>
#include <vector>

//! the volcap namespace
namespace volcap {

	//! the io namespace
	namespace io {

		class Camera {
		public:
			Eigen::Matrix3f orientation;
			Eigen::Vector3f location;
			Eigen::Matrix3f K;
			double height;
			double width;
			std::string name;

			Camera() {}
			Camera(std::string name, Eigen::Matrix3f &orientation, Eigen::Vector3f &location)
				: name(name), orientation(orientation), location(location) {}
			Camera(std::string name, Eigen::Matrix3f &orientation, Eigen::Vector3f &location,
				Eigen::Matrix3f &K, double height, double width)
				: name(name), orientation(orientation), location(location), K(K), height(height), width(width) {}

			Eigen::Vector3f getLocation() {
				return location;
			}

			Eigen::Matrix3f getOrientation() {
				return orientation;
			}

			Eigen::Affine3f getPose();

			//! converts to a PCL Camera object
			pcl::texture_mapping::Camera toPCLCamera();
		};

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
		 * @brief loads camera extrinsics / intrinsics output by VCL's VolumetricCapture
		 * @remark `calibration_dir` should contain a `*.extrinsics` file for each camera, and 
		 *  a `device_repository.json` containing all the intrisics
		 * @remark this function stores the intrinsics for the d415 RGB 1280*720 camera
		 * @todo this currently requires all `*.extrinsics` files to start with something alphabetically
		 *  less than `device_repository.json`. fix this.
		 *
		 * @param[in] calibration_dir	directory with necessary calibration files
		 * @param[out] cam_list			output vector to contain the cameras
		 */
		void loadCameraParams_VCL(
			std::string calibration_dir,
			std::vector<volcap::io::Camera*> &cam_list
		);

		/**
		 * @brief Manually sets a camera's params (for testing)
		 *
		 * @param[out] cam Camera that gets manually set.
		 */
		void hardcodeLoadCameraParam(pcl::TextureMapping<pcl::PointXYZ>::Camera &cam);
	}
}

#endif
