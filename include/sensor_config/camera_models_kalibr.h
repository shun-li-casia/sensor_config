/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: camera_models_kalibr.h
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

#include "camodocal/camera_models/PinholeCamera.h"
#include "utility_tool/pcm_debug_helper.h"

#include <vector>
#include <algorithm>
#include <string>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace sensor_config {
struct ImgImuConfig {
  Eigen::Matrix3d r_rl_;
  Eigen::Vector3d t_rl_;

  std::vector<camodocal::PinholeCamera::Parameters> cam_params_;

  std::vector<std::vector<int>> cam_overlaps_;
  std::vector<std::string> rostopic_;
};

class ConfigManager {
 public:
  static void ReadKalibr(const std::string calibn_file, ImgImuConfig* conf,
                         float resize_scale_factor = 1);

  static void WriteKalibr(const ImgImuConfig& conf,
                          const std::string file_name);
};
}  // namespace sensor_config
