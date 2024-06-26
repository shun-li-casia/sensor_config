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

  double& Imu::acc_n(){return acc_n_;}
  double& Imu::acc_w(){return acc_w_;}
  double& Imu::gyr_n(){return gyr_n_;}
  double& Imu::gyr_w(){return gyr_w_;}

  double Imu::acc_n() const{return acc_n_;}
  double Imu::acc_w() const{return acc_w_;}
  double Imu::gyr_n() const{return gyr_n_;}
  double Imu::gyr_w() const{return gyr_w_;}

  bool Imu::readFromKalibr(const std::string& path){return true;}
  bool Imu::readFromImuUtils(const std::string& path){return true;}
}
