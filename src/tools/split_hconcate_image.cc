/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: split_hconcate_image.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 28/05/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "utility_tool/cmdline.h"
#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/pcm_debug_helper.h"
#include "utility_tool/system_lib.h"

cv::Mat hconcate_img;
std_msgs ::Header hconcate_img_header;

image_transport::Publisher pub_left;
image_transport::Publisher pub_right;

int camera_id;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    hconcate_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    hconcate_img_header = msg->header;

  } catch (cv_bridge::Exception& e) {
    PCM_PRINT_ERROR("Could not convert from '%s' to 'bgr8'. \n",
                    msg->encoding.c_str());
  }

  int width = hconcate_img.cols;
  int height = hconcate_img.rows;

  cv::Rect leftRect(0, 0, width / 2, height);
  cv::Rect rightRect(width / 2, 0, width / 2, height);

  cv::Mat leftImage = hconcate_img(leftRect);
  cv::Mat rightImage = hconcate_img(rightRect);

  if (leftImage.empty() || rightImage.empty()) {
    PCM_PRINT_WARN("the left or right image is empty! \n");
    return;
  }
  std_msgs::Header l_header = hconcate_img_header;
  l_header.frame_id = "cam_" + std::to_string(camera_id) + "_img_0";
  std_msgs::Header r_header = hconcate_img_header;
  r_header.frame_id = "cam_" + std::to_string(camera_id) + "_img_1";

  sensor_msgs::ImagePtr left_msg =
      cv_bridge::CvImage(l_header, "bgr8", leftImage).toImageMsg();
  sensor_msgs::ImagePtr right_msg =
      cv_bridge::CvImage(r_header, "bgr8", rightImage).toImageMsg();

  pub_left.publish(left_msg);
  pub_right.publish(right_msg);
}

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<int>("camera_id", 'i', "camera id", false, 0);
  par.parse_check(argc, argv);
  camera_id = par.get<int>("camera_id");

  ros::init(argc, argv,
            "split_hconcate_image_node_" + std::to_string(camera_id));
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub = it.subscribe(
      "hconcate_image_cam_" + std::to_string(camera_id), 10, imageCallback);

  pub_left = it.advertise("cam_" + std::to_string(camera_id) + "_img_0", 10);
  pub_right = it.advertise("cam_" + std::to_string(camera_id) + "_img_1", 10);

  ros::spin();
  return 0;
}
