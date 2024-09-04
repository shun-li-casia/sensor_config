/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: record_start_end_odom.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 04/09/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <utility_tool/print_ctrl_macro.h>
#include <utility_tool/cmdline.h>
#include <utility_tool/system_lib.h>
#include <utility_tool/file_writter.h>

std::shared_ptr<utility_tool::FileWritter> file_writter;

void record_odom(const nav_msgs::Odometry::ConstPtr& msg) {
  double s = std::sqrt(msg->pose.pose.position.x * msg->pose.pose.position.x +
                       msg->pose.pose.position.y * msg->pose.pose.position.y +
                       msg->pose.pose.position.z * msg->pose.pose.position.z);

  // "tp", "x", "y", "z", "s", "qw","qx", "qy", "qz", "vx", "vy", "vz"
  file_writter->Write(msg->header.stamp.toSec(), msg->pose.pose.position.x,
                      msg->pose.pose.position.y, msg->pose.pose.position.z, s,
                      msg->pose.pose.orientation.w,
                      msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->twist.twist.linear.x,
                      msg->twist.twist.linear.y, msg->twist.twist.linear.z);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "record_start_end_odom");
  ros::NodeHandle nh;

  cmdline::parser par;
  par.add<int>("uav_id", 'i', "uav id", true);
  par.add<std::string>("file_path", 'f', "file_path", true);
  par.parse_check(argc, argv);

  std::string filename = par.get<std::string>("file_path") + "/" +
                         utility_tool::GetCurLocalTimeStr("%Y%m%d%H%M%S") +
                         "_uav_" + std::to_string(par.get<int>("uav_id")) +
                         "_odom.csv";
  file_writter = std::make_shared<utility_tool::FileWritter>(filename, 6);
  file_writter->SetDelimiter(",");
  file_writter->EraseOpen();
  // file_writter->Write("tp", "x", "y", "z", "s", "qx", "qy", "qz", "qw", "vx",
  // "vy", "vz");

  PCM_PRINT_INFO("record odom start! file save path: %s\n", filename.c_str());
  ros::Subscriber odom_sub = nh.subscribe("odometry", 100, record_odom);
  ros::spin();

  file_writter->Close();

  PCM_PRINT_INFO("record odom finish! file save path: %s\n", filename.c_str());
  return 0;
}
