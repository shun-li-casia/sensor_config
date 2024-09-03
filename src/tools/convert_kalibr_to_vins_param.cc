/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: convert_kalibr_to_vins_param.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 26/06/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "sensor_config/modules/stereo_imu_config.h"
#include "utility_tool/cmdline.h"

int main(int argc, char* argv[]) {
  cmdline::parser par;
  par.add<int>("uav_id", 0, "uav id", true);
  par.add<std::string>("k_imu", 0, "kalibr imu file", true);
  par.add<std::string>("k_cam_imu", 0, "kalibr camera imu file", true);
  par.add<float>("imu_noise_factor", 0, "imu noise factor", true, 1.0f);
  par.parse_check(argc, argv);

  std::string k_imu = par.get<std::string>("k_imu");
  std::string k_cam_imu = par.get<std::string>("k_cam_imu");

  int uav_id = par.get<int>("uav_id");

  sensor_config::StereoImu s(uav_id);
  s.readKalibr(k_cam_imu, k_imu);
  s.writeVins("uav_" + std::to_string(uav_id) + "_vins_params.yaml",
              par.get<int>("uav_id"), par.get<float>("imu_noise_factor"));

  return 0;
}
