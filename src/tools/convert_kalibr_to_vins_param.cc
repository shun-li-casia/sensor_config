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
  par.add<std::string>("k_imu", 0, "kalibr imu file", true);
  par.add<std::string>("k_cam_imu", 0, "kalibr camera imu file", true);
  par.parse_check(argc, argv);

  std::string k_imu = par.get<std::string>("k_imu");
  std::string k_cam_imu = par.get<std::string>("k_cam_imu");

  sensor_config::StereoImu s;
  s.readKalibr(k_cam_imu, k_imu);
  s.writeVins("vins_params.yaml");
}
