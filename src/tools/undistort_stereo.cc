/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: undistort_stereo.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 29/05/2024
 *
 *   @Description: subscribe the hconcated raw, undistort, pub the hconcate_rect
 *and left_rect
 *
 *******************************************************************************/

#include <ros/ros.h>

#include "cv_bridge/cv_bridge.h"
#include "sensor_config/modules/stereo_cam_config_manager.h"
#include "sensor_config/modules/stereo_rectifier.h"
#include "sensor_msgs/CameraInfo.h"
#include "utility_tool/cmdline.h"
#include "utility_tool/pcm_debug_helper.h"

#include <sensor_msgs/Image.h>
#include <thread>

int g_uav_id = -1;

// image
ros::Publisher g_l_rect_img_pub, g_r_rect_img_pub;

// camera info
ros::Publisher g_l_rect_info_pub, g_r_rect_info_pub;
sensor_msgs::CameraInfo g_l_rect_info, g_r_rect_info;

// the maps
std::pair<cv::Mat, cv::Mat> g_l_maps, g_r_maps;

void remap(const sensor_msgs::ImageConstPtr& msg,
           const std::pair<cv::Mat, cv::Mat>& maps, const std::string frame_id,
           ros::Publisher* pub) {
  try {
    // Convert the ROS Image message to OpenCV's Mat format
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat img = cv_ptr->image;

    // remap
    cv::Mat rect_img;
    cv::remap(img, rect_img, maps.first, maps.second, cv::INTER_LINEAR);

    // convert t
    cv_bridge::CvImage cv_img;
    cv_img.header = msg->header;
    cv_img.header.frame_id = frame_id;
    cv_img.encoding = sensor_msgs::image_encodings::BGR8;
    cv_img.image = rect_img;
    sensor_msgs::ImagePtr img_msg = cv_img.toImageMsg();
    pub->publish(img_msg);

  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void l_img_callback(const sensor_msgs::ImageConstPtr& msg) {
  remap(msg, g_l_maps, "uav_" + std::to_string(g_uav_id) + "_rect_cam_0",
        &g_l_rect_img_pub);
}

void r_img_callback(const sensor_msgs::ImageConstPtr& msg) {
  remap(msg, g_r_maps, "uav_" + std::to_string(g_uav_id) + "_rect_cam_1",
        &g_r_rect_img_pub);
}

void pub_rect_info() {
  ros::Rate rate(1);
  ros::Time tp = ros::Time::now();
  while (ros::ok()) {
    rate.sleep();
    g_l_rect_info.header.stamp = tp;
    g_l_rect_info_pub.publish(g_l_rect_info);

    g_r_rect_info.header.stamp = tp;
    g_r_rect_info_pub.publish(g_r_rect_info);
  }

  return;
}

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<int>("uav_id", 0, "uav id", true, 0);
  par.add<std::string>("k_stereo", 0, "kalibr stereo file", true);
  par.parse_check(argc, argv);
  g_uav_id = par.get<int>("uav_id");

  ros::init(argc, argv,
            "undistort_stereo_node_uav_" + std::to_string(g_uav_id));
  ros::NodeHandle nh;

  // sub the left and right camrea
  ros::Subscriber l_raw_sub = nh.subscribe<sensor_msgs::Image>(
      "uav_" + std::to_string(g_uav_id) + "/cam_0", 10, l_img_callback);
  ros::Subscriber r_raw_sub = nh.subscribe<sensor_msgs::Image>(
      "uav_" + std::to_string(g_uav_id) + "/cam_1", 10, r_img_callback);

  // pub the left and right camrea and rected image and info
  g_l_rect_img_pub = nh.advertise<sensor_msgs::Image>(
      "uav_" + std::to_string(g_uav_id) + "/rect/cam_0", 10);
  g_r_rect_img_pub = nh.advertise<sensor_msgs::Image>(
      "uav_" + std::to_string(g_uav_id) + "/rect/cam_1", 10);

  g_l_rect_info_pub = nh.advertise<sensor_msgs::CameraInfo>(
      "uav_" + std::to_string(g_uav_id) + "/rect/cam_0_info", 10);
  g_r_rect_info_pub = nh.advertise<sensor_msgs::CameraInfo>(
      "uav_" + std::to_string(g_uav_id) + "/rect/cam_1_info", 10);

  // apply the stereo
  sensor_config::StereoCamConfigManager stereo_conf_manager;
  sensor_config::StereoCamConfig stereo_conf;
  stereo_conf_manager.ReadKalibr(par.get<std::string>("k_stereo"),
                                 &stereo_conf);
  stereo_conf_manager.PrintKalibr(stereo_conf);

  Eigen::Matrix3d rect_r_rl;
  Eigen::Vector3d rect_t_rl;
  sensor_config::PinholeCamera::Parameters l_rect_cam =
                                               stereo_conf.cam_params_[0],
                                           r_rect_cam =
                                               stereo_conf.cam_params_[1];
  sensor_config::StereoRectifier::RectStereoParam(
      stereo_conf.r_rl_, stereo_conf.t_rl_, &rect_r_rl, &rect_t_rl, &l_rect_cam,
      &r_rect_cam, &g_l_maps, &g_r_maps);

  // left camera info
  g_l_rect_info.header.frame_id =
      "uav_" + std::to_string(g_uav_id) + "_rect_cam_0";
  g_l_rect_info.height = l_rect_cam.img_h();
  g_l_rect_info.width = l_rect_cam.img_w();
  g_l_rect_info.distortion_model = "plumb_bob";
  g_l_rect_info.K = {l_rect_cam.fx(),
                     0.0f,
                     l_rect_cam.cx(),
                     0.0f,
                     l_rect_cam.fy(),
                     l_rect_cam.cy(),
                     0.0f,
                     0.0f,
                     1.0f};
  g_l_rect_info.R = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  g_l_rect_info.D = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  g_l_rect_info.P = {l_rect_cam.fx(),
                     0.0f,
                     l_rect_cam.cx(),
                     0.0f,
                     0.0f,
                     l_rect_cam.fy(),
                     l_rect_cam.cy(),
                     0.0f,
                     0.0f,
                     0.0f,
                     1.0f,
                     0.0f};

  // right camera info
  g_r_rect_info.header.frame_id =
      "uav_" + std::to_string(g_uav_id) + "_rect_cam_1";
  g_r_rect_info.height = r_rect_cam.img_h();
  g_r_rect_info.width = r_rect_cam.img_w();
  g_r_rect_info.distortion_model = "plumb_bob";
  g_r_rect_info.K = {r_rect_cam.fx(),
                     0.0f,
                     r_rect_cam.cx(),
                     0.0f,
                     r_rect_cam.fy(),
                     r_rect_cam.cy(),
                     0.0f,
                     0.0f,
                     1.0f};
  g_r_rect_info.R = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  g_r_rect_info.D = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  Eigen::Matrix<double, 3, 4> T_rl;
  T_rl << rect_r_rl, rect_t_rl;
  Eigen::Matrix3d K_r;
  K_r << g_r_rect_info.K[0], g_r_rect_info.K[1], g_r_rect_info.K[2],
      g_r_rect_info.K[3], g_r_rect_info.K[4], g_r_rect_info.K[5],
      g_r_rect_info.K[6], g_r_rect_info.K[7], g_r_rect_info.K[8];

  Eigen::Matrix<double, 3, 4> P_r = K_r * T_rl;
  g_r_rect_info.P = {P_r(0, 0), P_r(0, 1), P_r(0, 2), P_r(0, 3),
                     P_r(1, 0), P_r(1, 1), P_r(1, 2), P_r(1, 3),
                     P_r(2, 0), P_r(2, 1), P_r(2, 2), P_r(2, 3)};

  std::thread t_info(pub_rect_info);
  ros::spin();
  t_info.join();

  return 0;
}
