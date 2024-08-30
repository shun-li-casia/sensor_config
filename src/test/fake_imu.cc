/*******************************************************************************
 *   Copyright (C) 2024. All rights reserved.
 *
 *   @Filename: fake_imu.cc
 *
 *   @Author: ShunLi
 *
 *   @Email: is.shun.li@outlook.com
 *
 *   @Date: 30/08/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include <utility_tool/print_ctrl_macro.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

double uniform_rand(double lowerBndr, double upperBndr) {
  return lowerBndr +
         ((double)std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}
double gauss_rand(double mean, double sigma) {
  double x, y, r2;
  do {
    x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    r2 = x * x + y * y;
  } while (r2 > 1.0 || r2 == 0.0);
  return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "fake_imu_node");
  ros::NodeHandle nh;
  ros::Publisher imu_pub =
      nh.advertise<sensor_msgs::Imu>("/uav_2/imu_raw_0", 2000);

  while (ros::ok()) {
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "uav_2_imu_raw_0";
    imu.orientation.x = 0;
    imu.orientation.y = 0;
    imu.orientation.z = 0;
    imu.orientation.w = 0;
    imu.angular_velocity.x = gauss_rand(1, 0.1);
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;
    imu.linear_acceleration.x = gauss_rand(1, 0.1);
    imu.linear_acceleration.y = 0;
    imu.linear_acceleration.z = 0;
    imu_pub.publish(imu);
    ros::Rate(200).sleep();
  }
  return 0;
}
