/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: save_rosimg_to_file.cc
 *
 *   @Author: ShunLi
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 26/04/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include "utility_tool/pcm_debug_helper.h"
#include "utility_tool/system_lib.h"
#include "utility_tool/cmdline.h"

#include <ros/init.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

void img_callback(const sensor_msgs::Image::ConstPtr& img_msg) {
  cv_bridge::CvImageConstPtr ptr =
      cv_bridge::toCvCopy(img_msg, img_msg->encoding);
  const cv::Mat& rgb = ptr->image;
  cv::imshow("usb_cam_img", rgb);
  cv::waitKey(1);

  std::string cur_t = std::to_string(img_msg->header.stamp.toNSec());
  cv::imwrite(cur_t + ".png", rgb);
  PCM_PRINT_INFO("image received! %s\n", (cur_t + ".png").c_str());
}

int main(int argc, char* argv[]) {
  cmdline::parser par;

  par.add<std::string>("image_topic", 'i', "image topic", true);
  par.parse_check(argc, argv);
  std::string image_topic = par.get<std::string>("image_topic");

  std::string t = utility_tool::GetCurLocalTimeStr("%Y%m%d%H%M%S");
  ros::init(argc, argv, "save_rosimg_to_file_node_" + t);
  ros::NodeHandle nh;
  ros::Subscriber img_sub = nh.subscribe(image_topic, 100, img_callback);

  ros::spin();
  return 0;
}
