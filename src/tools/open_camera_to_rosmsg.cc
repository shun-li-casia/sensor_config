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

#include <ros/ros.h>
#include <ros/duration.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
  cmdline::parser par;

  par.add<int>("camera_id", 'c', "camera id", true);
  par.add<int>("pub_downsample_rate", 'r', "publish frequency rate", true, 1);
  par.parse_check(argc, argv);
  int camera_id = par.get<int>("camera_id");
  int pub_downsample_rate = par.get<int>("pub_downsample_rate");

  if (pub_downsample_rate < 1) {
    PCM_PRINT_WARN("pub_downsample_rate should be greater or equal than 1! \n");
    pub_downsample_rate = 1;
  }

  if (25 % pub_downsample_rate != 0) {
    PCM_PRINT_WARN("pub_downsample_rate should be a divisor of 25! \n");
    pub_downsample_rate = 1;
  }

  PCM_PRINT_INFO("camera_id: %d, pub_downsample_rate: %d \n", camera_id,
                 pub_downsample_rate);

  ros::init(argc, argv,
            "open_camera_" + std::to_string(camera_id) + "_to_rosmsg_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  image_transport::Publisher image_pub =
      it.advertise("hconcate_image_cam_" + std::to_string(camera_id), 10);
  image_transport::Publisher pub_left =
      it.advertise("cam_" + std::to_string(camera_id) + "_img_0", 10);
  image_transport::Publisher pub_right =
      it.advertise("cam_" + std::to_string(camera_id) + "_img_1", 10);

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

  const int round = 25 / pub_downsample_rate;
  int count = 0;
  while (ros::ok()) {
    timer.Start();
    cv::Mat frame, rgb;
    cap >> frame;
    if (frame.empty()) {
      PCM_PRINT_WARN("frame is empty!\n");
      continue;
    }
    cv::cvtColor(frame, rgb, cv::COLOR_YUV2BGR_UYVY);

    int width = rgb.cols;
    int height = rgb.rows;

    cv::Rect leftRect(0, 0, width / 2, height);
    cv::Rect rightRect(width / 2, 0, width / 2, height);

    cv::Mat leftImage = rgb(leftRect);
    cv::Mat rightImage = rgb(rightRect);

    if (leftImage.empty() || rightImage.empty()) {
      PCM_PRINT_WARN("the left or right image is empty! \n");
      continue;
    }

    std_msgs::Header header;
    header.frame_id = "cam_" + std::to_string(camera_id);
    header.stamp = ros::Time::now();

    std_msgs::Header l_header = header;
    l_header.frame_id = "cam_" + std::to_string(camera_id) + "_img_0";
    std_msgs::Header r_header = header;
    r_header.frame_id = "cam_" + std::to_string(camera_id) + "_img_1";

    // pub as ros msg
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(header, "bgr8", rgb).toImageMsg();
    sensor_msgs::ImagePtr left_msg =
        cv_bridge::CvImage(l_header, "bgr8", leftImage).toImageMsg();
    sensor_msgs::ImagePtr right_msg =
        cv_bridge::CvImage(r_header, "bgr8", rightImage).toImageMsg();

    if (++count == round) {
      count = 0;
      image_pub.publish(msg);
      pub_left.publish(left_msg);
      pub_right.publish(right_msg);
    }

    PCM_PRINT_INFO("loop cost: %.2f ms(%.2f Hz), total cost: %.2f s\n",
                   timer.End(), 1000.0 / timer.End(), total.End() / 1000);
  }

  cap.release();
  PCM_PRINT_INFO("the node is shutdown!\n");
  return 0;
}
