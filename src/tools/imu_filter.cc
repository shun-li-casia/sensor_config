/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: imu_filter.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 29/08/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "sensor_config/modules/imu_filter/one_order_low_pass.h"
#include <utility_tool/print_ctrl_macro.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>

std::shared_ptr<OneOrderLowPassFilter<Eigen::Vector3d>> acc_filter;
std::shared_ptr<OneOrderLowPassFilter<Eigen::Vector3d>> gyr_filter;
ros::Publisher imu_pub;

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  Eigen::Vector3d acc(imu_msg->linear_acceleration.x,
                      imu_msg->linear_acceleration.y,
                      imu_msg->linear_acceleration.z);
  Eigen::Vector3d gyr(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
                      imu_msg->angular_velocity.z);
  Eigen::Vector3d acc_new = acc_filter->GetOutput(acc);
  Eigen::Vector3d gyr_new = gyr_filter->GetOutput(gyr);

  sensor_msgs::Imu imu = *imu_msg;
  imu.linear_acceleration.x = acc_new(0);
  imu.linear_acceleration.y = acc_new(1);
  imu.linear_acceleration.z = acc_new(2);
  imu.angular_velocity.x = gyr_new(0);
  imu.angular_velocity.y = gyr_new(1);
  imu.angular_velocity.z = gyr_new(2);

  imu_pub.publish(imu);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "imu_filter");
  ros::NodeHandle nh;

  std::string in_imu_topic, out_imu_topic;
  float gain;
  nh.param("in_imu_topic", in_imu_topic, std::string("/uav_2/imu_raw_0"));
  nh.param("out_imu_topic", out_imu_topic, std::string("/uav_2/imu_filter"));
  nh.param("gain", gain, float(0.1));

  acc_filter = std::make_shared<OneOrderLowPassFilter<Eigen::Vector3d>>(
      Eigen::Vector3d(0, 0, 0), gain);
  gyr_filter = std::make_shared<OneOrderLowPassFilter<Eigen::Vector3d>>(
      Eigen::Vector3d(0, 0, 0), gain);

  imu_pub = nh.advertise<sensor_msgs::Imu>(out_imu_topic, 2000);
  ros::Subscriber imu_sub = nh.subscribe(in_imu_topic, 2000, imu_callback);

  PCM_PRINT_INFO("start imu filter!\n");

  ros::spin();

  return 0;
}
