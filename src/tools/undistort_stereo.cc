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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <thread>

#include <image_transport/image_transport.h>

int g_uav_id = -1;
int g_pub_rate = 25;

// image
image_transport::Publisher g_l_rect_img_pub, g_r_rect_img_pub;

// camera info
ros::Publisher g_l_rect_info_pub, g_r_rect_info_pub;
sensor_msgs::CameraInfo g_l_rect_info, g_r_rect_info;

// the maps
std::pair<cv::Mat, cv::Mat> g_l_maps, g_r_maps;

void remap(const sensor_msgs::ImageConstPtr& img_msg,
           sensor_msgs::CameraInfo* cam_msg,
           const std::pair<cv::Mat, cv::Mat>& maps, const std::string frame_id,
           image_transport::Publisher* img_pub, ros::Publisher* cam_pub) {
  try {
    // Convert the ROS Image message to OpenCV's Mat format
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat img = cv_ptr->image;

    // remap
    cv::Mat rect_img;
    cv::remap(img, rect_img, maps.first, maps.second, cv::INTER_LINEAR);

    // convert t
    cv_bridge::CvImage cv_img;
    cv_img.header = img_msg->header;
    cv_img.header.frame_id = frame_id;
    cv_img.encoding = sensor_msgs::image_encodings::MONO8;
    cv_img.image = rect_img;
    sensor_msgs::ImagePtr img_msg = cv_img.toImageMsg();
    img_pub->publish(img_msg);

    // publish the cam_info_msg
    cam_msg->header = img_msg->header;
    cam_pub->publish(*cam_msg);

  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

int g_pub_cnt = 0;
void imageCallback(const sensor_msgs::ImageConstPtr& image0,
                   const sensor_msgs::ImageConstPtr& image1) {
  if (++g_pub_cnt != g_pub_rate) return;

  g_pub_cnt = 0;
  try {
    // remap(image0, &g_l_rect_info, g_l_maps,"uav_" + std::to_string(g_uav_id)
    // + "rect_cam_0", &g_l_rect_img_pub, &g_l_rect_info_pub);
    std::thread t0(remap, image0, &g_l_rect_info, g_l_maps,
                   "uav_" + std::to_string(g_uav_id), &g_l_rect_img_pub,
                   &g_l_rect_info_pub);
    std::thread t1(remap, image1, &g_r_rect_info, g_r_maps,
                   "uav_" + std::to_string(g_uav_id), &g_r_rect_img_pub,
                   &g_r_rect_info_pub);
    t0.join();
    t1.join();
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<int>("uav_id", 0, "uav id", true, 0);
  par.add<std::string>("k_stereo", 0, "kalibr stereo file", true);
  par.add<int>("pub_rate", 0, "publish rate", true, 0);
  par.parse_check(argc, argv);
  g_uav_id = par.get<int>("uav_id");
  g_pub_rate = par.get<int>("pub_rate");

  ros::init(argc, argv,
            "undistort_stereo_node_uav_" + std::to_string(g_uav_id));
  ros::NodeHandle nh;

  // sub the left and right camrea
  message_filters::Subscriber<sensor_msgs::Image> image_sub1(nh, "cam_0", 10);
  message_filters::Subscriber<sensor_msgs::Image> image_sub2(nh, "cam_1", 10);
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                    sensor_msgs::Image>
      MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub1,
                                                   image_sub2);
  sync.registerCallback(boost::bind(&imageCallback, boost::placeholders::_1,
                                    boost::placeholders::_2));

  // pub the left and right camrea and rected image and info
  image_transport::ImageTransport it(nh);
  g_l_rect_img_pub = it.advertise("rect/cam_0", 100);
  g_r_rect_img_pub = it.advertise("rect/cam_1", 100);

  g_l_rect_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("rect/cam_0_info", 100);
  g_r_rect_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("rect/cam_1_info", 100);

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
  g_r_rect_info.K = {l_rect_cam.fx(),
                     0.0f,
                     r_rect_cam.cx(),
                     0.0f,
                     l_rect_cam.fy(),
                     l_rect_cam.cy(),
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
  g_r_rect_info.P = {K_r(0, 0), K_r(0, 1), K_r(0, 2), P_r(0, 3),
                     K_r(1, 0), K_r(1, 1), K_r(1, 2), P_r(1, 3),
                     K_r(2, 0), K_r(2, 1), K_r(2, 2), P_r(2, 3)};

  ros::spin();

  return 0;
}
