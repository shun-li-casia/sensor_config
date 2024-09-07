/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: tp_checker.cc
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

#include <utility_tool/cmdline.h>
#include <sensor_config/tools/tp_checker.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<std::string>("type", 'i', "file name", true);
  par.add<std::string>("topic", 't', "file name", true);

  par.parse_check(argc, argv);

  std::string topic = par.get<std::string>("topic");
  std::string type = par.get<std::string>("type");
  if (type == "imu") {
    sensor_config::TpChecker<sensor_msgs::Imu> tp_checker(topic);
    tp_checker.Spin();
  } else if (type == "image") {
    sensor_config::TpChecker<sensor_msgs::Image> tp_checker(topic);
    tp_checker.Spin();
  } else {
    PCM_PRINT_ERROR("Type %s is not supported", type.c_str());
    return -1;
  }

  return 0;
}
