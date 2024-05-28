/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: open_camera_to_rosmsg.h
 *
 *   @Author: ShunLi
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 12/04/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "opencv2/imgproc.hpp"
#include "utility_tool/cmdline.h"
#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/pcm_debug_helper.h"
#include "utility_tool/system_lib.h"

#include <ros/duration.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
  cmdline::parser par;

  par.add<int>("camera_id", 'i', "camera id", false, 0);
  par.parse_check(argc, argv);
  int camera_id = par.get<int>("camera_id");

  ros::init(argc, argv,
            "open_camera_" + std::to_string(camera_id) + "_to_rosmsg_node");
  ros::NodeHandle nh;

  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>(
      "hconcate_image_cam_" + std::to_string(camera_id), 1);

  cv::VideoCapture cap;
  for (int i = 0; i < 10 && ros::ok(); ++i) {
    cap.open(camera_id, cv::CAP_V4L2);
    bool is_open = cap.isOpened();
    if (is_open) {
      PCM_PRINT_INFO("open the camera successfully!\n");
      break;
    }
    if (!is_open && i == 9) {
      PCM_PRINT_ERROR("can not open the camera after %d times! Exit!\n", i + 1);
      return -1;
    }

    PCM_PRINT_INFO("tryed %d times to open the camera.\n", i + 1);
    ros::Duration(1).sleep();
  }

  cap.set(cv::CAP_PROP_CONVERT_RGB, 0);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 3840);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
  cap.set(cv::CAP_PROP_FPS, 25);

  if (!cap.isOpened()) {
    PCM_PRINT_ERROR("can not open the camera!\n");
    return -1;
  } else {
    PCM_PRINT_INFO("open the camera successfully!\n");
  }

  utility_tool::Timer timer, total;
  total.Start();
  while (ros::ok()) {
    timer.Start();
    cv::Mat frame, rgb;
    cap >> frame;
    if (frame.empty()) {
      PCM_PRINT_WARN("frame is empty!\n");
      continue;
    }
    cv::cvtColor(frame, rgb, cv::COLOR_YUV2BGR_UYVY);

    // pub as ros msg
    std_msgs::Header header;
    header.frame_id = "camera_" + std::to_string(camera_id);
    header.stamp = ros::Time::now();
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(header, "bgr8", rgb).toImageMsg();
    image_pub.publish(msg);
    PCM_PRINT_INFO("loop cost: %.2f ms(%.2f Hz), total cost: %.2f s\n",
                   timer.End(), 1000.0 / timer.End(), total.End() / 1000);
  }

  cap.release();
  PCM_PRINT_INFO("the node is shutdown!\n");
  return 0;
}
