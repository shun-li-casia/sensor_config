/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: test_extreme_ros_time.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 07/09/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include <ros/ros.h>
#include <utility_tool/print_ctrl_macro.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_ros_time_node");
  ros::NodeHandle nh;

  ros::Time last_time(0);
  while (ros::ok()) {
    ros::Time now = ros::Time::now();
    float time_diff = (now - last_time).toSec();
    float hz = 1.0 / time_diff;
    PCM_PRINT_INFO("time diff: %f, hz: %f\n", time_diff, hz);
    last_time = now;
  }

  return 0;
}
