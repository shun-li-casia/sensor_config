/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: imu.h
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

#ifndef SENSOR_CONFIG_IMU_MODEL_IMU_H_
#define SENSOR_CONFIG_IMU_MODEL_IMU_H_

#include <string>

namespace sensor_config {
class Imu {
 public:
  double& acc_n();
  double& acc_w();
  double& gyr_n();
  double& gyr_w();
  double& hz();
  std::string& rostopic();

  double acc_n() const;
  double acc_w() const;
  double gyr_n() const;
  double gyr_w() const;
  double hz() const;
  std::string rostopic() const;

  bool readFromKalibr(const std::string& path);
  bool readFromImuUtils(const std::string& path);

 private:
  double acc_n_{0.0f};
  double acc_w_{0.0f};
  double gyr_n_{0.0f};
  double gyr_w_{0.0f};

  std::string rostopic_{""};
  double hz_{0.0f};
};
}  // namespace sensor_config

#endif
