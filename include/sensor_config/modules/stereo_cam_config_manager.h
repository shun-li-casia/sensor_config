/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: stereo_cam_config_manager.h
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 09/10/2023
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef SENSOR_CONFIG_MODULES_STEREO_CAM_CONFIG_MANAGER_H_
#define SENSOR_CONFIG_MODULES_STEREO_CAM_CONFIG_MANAGER_H_

#include "sensor_config/camera_model/pinhole_camera.h"
#include "utility_tool/pcm_debug_helper.h"

#include <vector>
#include <algorithm>
#include <string>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace sensor_config {
struct StereoCamConfig {
  Eigen::Matrix3d r_rl_;
  Eigen::Vector3d t_rl_;

  std::vector<PinholeCamera::Parameters> cam_params_;

  std::vector<std::vector<int>> cam_overlaps_;
  std::vector<std::string> rostopic_;
};

class StereoCamConfigManager {
 public:
  static void ReadKalibr(const std::string calibn_file, StereoCamConfig* conf,
                         float resize_scale_factor = 1);

  static void WriteKalibr(const StereoCamConfig& conf,
                          const std::string file_name);
};
}  // namespace sensor_config
#endif
