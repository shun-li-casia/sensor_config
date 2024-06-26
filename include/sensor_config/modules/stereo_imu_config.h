/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: stereo_imu_config.h
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

#ifndef SENSOR_CONFIG_MODULES_STEREO_IMU_CONFIG_H_
#define SENSOR_CONFIG_MODULES_STEREO_IMU_CONFIG_H_

#include "sensor_config/camera_model/pinhole_camera.h"
#include "sensor_config/imu_model/imu.h"
#include <Eigen/Core>

namespace sensor_config {
class StereoImu {
 public:
  class CamInStereo {
   public:
    Eigen::Matrix4d Tci_;
    double time_shift_;
    PinholeCamera::Parameters cam_params_;
  };

  bool readKalibr(const std::string& cam_imu_chain_path, const std::string& imu_path);

  bool writeVins(const std::string& path) const;

 private:
  CamInStereo cam0_, cam1_;
  Eigen::Matrix4d T_c1_c0_;
  Imu imu0_;

  void writeCameraCV(const CamInStereo& cam) const;
};
}  // namespace sensor_config

#endif
