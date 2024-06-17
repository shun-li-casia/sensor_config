/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: preview_rosimage.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 17/06/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>

#include "utility_tool/cmdline.h"
#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/pcm_debug_helper.h"

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    // Convert the ROS image message to OpenCV format using cv_bridge
    cv::Mat cv_image =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    // Calculate the new size, halving both width and height
    cv::Size new_size(cv_image.cols / 2, cv_image.rows / 2);

    // Resize the image
    cv::Mat resized_image;
    cv::resize(cv_image, resized_image, new_size, 0, 0,
               cv::INTER_LINEAR);  // Use linear interpolation for resizing

    // Display the resized image
    cv::imshow("Resized Image Window", resized_image);
    cv::waitKey(
        30);  // Wait for 30ms to prevent the program from exiting immediately
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Conversion from '%s' to 'bgr8' failed.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv) {

  cmdline::parser par;
  par.add<std::string>("image_topic", 'i', "image topic", true);
  par.parse_check(argc, argv);

  // Initialize the ROS node
  ros::init(argc, argv, "preview_rosimage_node");
  ros::NodeHandle nh;

  std::string image_topic = par.get<std::string>("image_topic");

  // Create an Image Transport object
  image_transport::ImageTransport it(nh);

  // Subscribe to the image topic, e.g., "/camera/image_raw"; adjust as needed
  image_transport::Subscriber sub =
      it.subscribe(image_topic, 1, imageCallback);

  // Spin to start the ROS event processing loop
  ros::spin();

  return 0;
}
