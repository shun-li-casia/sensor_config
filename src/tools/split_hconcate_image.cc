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

cv::Mat hconcate_img;
std_msgs ::Header hconcate_img_header;

image_transport::Publisher pub_left;
image_transport::Publisher pub_right;

int uav_id, camera_id;
int rate_cnt = 0, callback_cnt = 0;
int img_seq = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  if (++callback_cnt != rate_cnt) return;
  callback_cnt = 0;

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
  l_header.frame_id = "uav_" + std::to_string(uav_id) + "_cam_0";
  l_header.seq = img_seq;
  std_msgs::Header r_header = hconcate_img_header;
  r_header.frame_id = "uav_" + std::to_string(uav_id) + "_cam_1";
  r_header.seq = img_seq++;

  sensor_msgs::ImagePtr left_msg =
      cv_bridge::CvImage(l_header, "bgr8", leftImage).toImageMsg();
  sensor_msgs::ImagePtr right_msg =
      cv_bridge::CvImage(r_header, "bgr8", rightImage).toImageMsg();

  pub_left.publish(left_msg);
  pub_right.publish(right_msg);
}

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<int>("uav_id", 'u', "uav id", true);
  par.add<int>("camera_id", 'c', "camera id", true, 0);
  par.add<int>("publish_rate", 'r', "publish rate", true, 0);
  par.parse_check(argc, argv);
  uav_id = par.get<int>("uav_id");
  camera_id = par.get<int>("camera_id");

  ros::init(argc, argv,
            "split_hconcate_image_node_" + std::to_string(camera_id));
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub = it.subscribe(
      "hconcate_image_cam_" + std::to_string(camera_id), 10, imageCallback);

  std::string left_topic = "uav_" + std::to_string(uav_id) + "/_cam_0";
  std::string right_topic = "uav_" + std::to_string(uav_id) + "/_cam_1";

  pub_left = it.advertise(left_topic, 10);
  pub_right = it.advertise(right_topic, 10);

  int rate = par.get<int>("publish_rate");
  PCM_PRINT_INFO("camera_id: %d, publish_rate: %d \n", camera_id, rate);
  if (rate < 1) {
    PCM_PRINT_WARN("publish_rate should be greater or equal than 1! \n");
    rate = 1;
  } else if (rate > 25) {
    PCM_PRINT_WARN("publish_rate should be less or equal than 25! \n");
    rate = 25;
  }

  rate_cnt = 25 / rate;

  PCM_PRINT_INFO("start to publish left and right image on %s and %s \n",
                 left_topic.c_str(), right_topic.c_str());
  ros::spin();
  return 0;
}
