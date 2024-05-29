/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: undistort_img.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 29/05/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "utility_tool/cmdline.h"
#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/pcm_debug_helper.h"
#include "utility_tool/system_lib.h"

#include "sensor_config/camera_model/pinhole_camera.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>

cv::Mat raw_img;
std_msgs ::Header header;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    raw_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    header = msg->header;

  } catch (cv_bridge::Exception& e) {
    PCM_PRINT_ERROR("Could not convert from '%s' to 'bgr8'. \n",
                    msg->encoding.c_str());
  }
}

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<std::string>("cam_kalibr_file", 'k', "cam_kalibr_file", true);
  par.parse_check(argc, argv);
  std::string cam_kalibr_file = par.get<std::string>("cam_kalibr_file");

  sensor_config::PinholeCamera::Parameters params, undis_params;
  if (!params.readKalibrSingleCam(cam_kalibr_file)) {
    PCM_PRINT_ERROR("readKalibrSingleCam failed! \n");
    return -1;
  }

  sensor_config::PinholeCamera camera(params);

  std::pair<cv::Mat, cv::Mat> maps;
  camera.RectSignalCamParam(&undis_params, &maps);
  undis_params.writeKalibrSingleCam(undis_params.camera_name() + ".yaml");

  ros::init(argc, argv, "undistort_img");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub =
      it.subscribe(params.rostopic(), 10, imageCallback);
  image_transport::Publisher pub = it.advertise(undis_params.rostopic(), 10);

  ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>(
      undis_params.camera_name() + "/camera_info", 10);

  while (ros::ok()) {
    ros::spinOnce();
    if (raw_img.empty()) {
      continue;
    }
    header.frame_id = header.frame_id + "_rect";

    // cam info
    sensor_msgs::CameraInfo camera_info;
    camera_info.header = header;
    camera_info.height = raw_img.rows;
    camera_info.width = raw_img.cols;
    camera_info.distortion_model = "plumb_bob";
    camera_info.D = {0, 0, 0, 0, 0};
    camera_info.K = {
        undis_params.fx(),
        0,
        undis_params.cx(),
        0,
        undis_params.fy(),
        undis_params.cy(),
        0,
        0,
        1,
    };
    camera_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    camera_info.P = {
        undis_params.fx(),
        0,
        undis_params.cx(),
        0,
        0,
        undis_params.fy(),
        undis_params.cy(),
        0,
        0,
        0,
        1,
        0,
    };
    camera_info_pub.publish(camera_info);

    // rected image
    cv::Mat undistorted_img;
    cv::remap(raw_img, undistorted_img, maps.first, maps.second,
              cv::INTER_LINEAR);
    sensor_msgs::ImagePtr undistorted_msg =
        cv_bridge::CvImage(header, "bgr8", undistorted_img).toImageMsg();
    pub.publish(undistorted_msg);
  }
}
