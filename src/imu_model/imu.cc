/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: imu.cc
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

#include "sensor_config/imu_model/imu.h"
#include <yaml-cpp/yaml.h>

namespace sensor_config {

double& Imu::acc_n() { return acc_n_; }
double& Imu::acc_w() { return acc_w_; }
double& Imu::gyr_n() { return gyr_n_; }
double& Imu::gyr_w() { return gyr_w_; }
double& Imu::hz() { return hz_; }
std::string& Imu::rostopic() { return rostopic_; }

double Imu::acc_n() const { return acc_n_; }
double Imu::acc_w() const { return acc_w_; }
double Imu::gyr_n() const { return gyr_n_; }
double Imu::gyr_w() const { return gyr_w_; }
double Imu::hz() const { return hz_; }
std::string Imu::rostopic() const { return rostopic_; }

bool Imu::readFromKalibr(const std::string& path) {
  auto n = YAML::LoadFile(path);
  if (n.IsNull()) {
    return false;
  }

  double acc_n = n["accelerometer_noise_density"].as<double>();
  acc_n_ = acc_n;

  double acc_w = n["accelerometer_random_walk"].as<double>();
  acc_w_ = acc_w;

  double gyr_n = n["gyroscope_noise_density"].as<double>();
  gyr_n_ = gyr_n;

  double gyr_w = n["gyroscope_random_walk"].as<double>();
  gyr_w_ = gyr_w;

  std::string rostopic = n["rostopic"].as<std::string>();
  rostopic_ = rostopic;

  double hz = n["update_rate"].as<double>();
  hz_ = hz;

  return true;
}

bool Imu::readFromImuUtils(const std::string& path) { return true; }
}  // namespace sensor_config
