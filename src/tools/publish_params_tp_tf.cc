/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: publish_params_tp_tf.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 13/07/2024
 *
 *   @Description: read the kailbr file, publish the camera and imu configs
 *
 *******************************************************************************/

#include "sensor_config/modules/stereo_imu_config.h"
#include "sensor_config/apriltag_model/tag_extrin_model.h"
#include "sensor_msgs/CameraInfo.h"
#include "utility_tool/cmdline.h"

#include <ros/ros.h>
#include <sophus/se3.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char** argv) {
  cmdline::parser par;
  par.add<int>("uav_id", 0, "uav id", true);
  par.add<std::string>(
      "t_b_file", 0,
      "the transformation between the tag and the body, use none if not exist",
      true);
  par.add<std::string>("k_imu", 0, "kalibr imu file", true);
  par.add<std::string>("k_cam_imu", 0, "kalibr camera imu file", true);
  par.parse_check(argc, argv);

  const int uav_id = par.get<int>("uav_id");
  const std::string imu_file = par.get<std::string>("k_imu");
  const std::string cam_imu_file = par.get<std::string>("k_cam_imu");

  std::string t_b_t_file = par.get<std::string>("t_b_file");
  bool has_tag = (t_b_t_file != "none") && (t_b_t_file != "");

  ros::init(argc, argv, "publish_params_tp_tf_node");
  ros::NodeHandle nh;

  sensor_config::StereoImu stereo_imu(uav_id);
  stereo_imu.readKalibr(cam_imu_file, imu_file);

  // STEP: brodacast the tf, T_b_c0, T_b_c1, T_b_t (only uav_id=1,2,3)
  tf2_ros::StaticTransformBroadcaster tf_br;
  auto tp = ros::Time::now();

  // STEP: T_b_c0
  geometry_msgs::TransformStamped trans_b_c0;
  trans_b_c0.header.stamp = tp;
  trans_b_c0.header.frame_id = "uav_" + std::to_string(uav_id) + "_b";
  trans_b_c0.child_frame_id = "uav_" + std::to_string(uav_id) + "_cam_0";

  Sophus::SE3d T_c0_b(stereo_imu.cam0_.Tci_);
  Sophus::SE3d T_b_c0 = T_c0_b.inverse();
  std::cout << "T_c0_b:\n" << T_c0_b.matrix() << std::endl;
  std::cout << "T_b_c0:\n" << T_b_c0.matrix() << std::endl;
  trans_b_c0.transform.translation.x = T_b_c0.translation().x();
  trans_b_c0.transform.translation.y = T_b_c0.translation().y();
  trans_b_c0.transform.translation.z = T_b_c0.translation().z();
  trans_b_c0.transform.rotation.w = T_b_c0.unit_quaternion().w();
  trans_b_c0.transform.rotation.x = T_b_c0.unit_quaternion().x();
  trans_b_c0.transform.rotation.y = T_b_c0.unit_quaternion().y();
  trans_b_c0.transform.rotation.z = T_b_c0.unit_quaternion().z();

  tf_br.sendTransform(trans_b_c0);

  // STEP: T_b_c1
  geometry_msgs::TransformStamped trans_b_c1;
  trans_b_c1.header.stamp = tp;
  trans_b_c1.header.frame_id = "uav_" + std::to_string(uav_id) + "_b";
  trans_b_c1.child_frame_id = "uav_" + std::to_string(uav_id) + "_cam_1";

  Sophus::SE3d T_c1_b(stereo_imu.cam1_.Tci_);
  Sophus::SE3d T_b_c1 = T_c1_b.inverse();
  trans_b_c1.transform.translation.x = T_b_c1.translation().x();
  trans_b_c1.transform.translation.y = T_b_c1.translation().y();
  trans_b_c1.transform.translation.z = T_b_c1.translation().z();
  trans_b_c1.transform.rotation.w = T_b_c1.unit_quaternion().w();
  trans_b_c1.transform.rotation.x = T_b_c1.unit_quaternion().x();
  trans_b_c1.transform.rotation.y = T_b_c1.unit_quaternion().y();
  trans_b_c1.transform.rotation.z = T_b_c1.unit_quaternion().z();
  tf_br.sendTransform(trans_b_c1);

  // STEP: T_b_t
  geometry_msgs::TransformStamped trans_b_t;
  if (has_tag) {
    TagExtrinModel tag_extrin;
    if (!tag_extrin.readTagExtrin(t_b_t_file)) {
      ROS_ERROR("read tag extrin error");
      return -1;
    } else {
      trans_b_t = tag_extrin.T_b_t_;
      trans_b_t.header.stamp = tp;
      trans_b_t.header.frame_id = "uav_" + std::to_string(uav_id) + "_b";
      trans_b_t.child_frame_id = "uav_" + std::to_string(uav_id) + "_tag";

      tf_br.sendTransform(trans_b_t);
    }
  }

  // STEP: camera 0
  ros::Publisher cam_0_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("cam_0_info", 10);
  sensor_msgs::CameraInfo cam_0_info;
  cam_0_info.header.frame_id = "uav_" + std::to_string(uav_id) + "_cam_0";
  cam_0_info.width = stereo_imu.cam0_.cam_params_.img_w();
  cam_0_info.height = stereo_imu.cam0_.cam_params_.img_h();
  cam_0_info.distortion_model = "plumb_bob";
  cam_0_info.D = {stereo_imu.cam0_.cam_params_.k1(),
                  stereo_imu.cam0_.cam_params_.k2(),
                  stereo_imu.cam0_.cam_params_.p1(),
                  stereo_imu.cam0_.cam_params_.p2(), 0.0f};
  cam_0_info.K = {stereo_imu.cam0_.cam_params_.fx(),
                  0.0f,
                  stereo_imu.cam0_.cam_params_.cx(),
                  0.0f,
                  stereo_imu.cam0_.cam_params_.fy(),
                  stereo_imu.cam0_.cam_params_.cy(),
                  0.0f,
                  0.0f,
                  1.0f};
  cam_0_info.R = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  cam_0_info.P = {stereo_imu.cam0_.cam_params_.fx(),
                  0.0f,
                  stereo_imu.cam0_.cam_params_.cx(),
                  0.0f,
                  0.0f,
                  stereo_imu.cam0_.cam_params_.fy(),
                  stereo_imu.cam0_.cam_params_.cy(),
                  0.0f,
                  0.0f,
                  0.0f,
                  1.0f,
                  0.0f};

  // STEP: camera 1
  sensor_msgs::CameraInfo cam_1_info;
  ros::Publisher cam_1_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("cam_1_info", 10);
  cam_1_info.header.frame_id = "uav_" + std::to_string(uav_id) + "_cam_1";
  cam_1_info.width = stereo_imu.cam1_.cam_params_.img_w();
  cam_1_info.height = stereo_imu.cam1_.cam_params_.img_h();
  cam_1_info.distortion_model = "plumb_bob";
  cam_1_info.D = {stereo_imu.cam1_.cam_params_.k1(),
                  stereo_imu.cam1_.cam_params_.k2(),
                  stereo_imu.cam1_.cam_params_.p1(),
                  stereo_imu.cam1_.cam_params_.p2(), 0.0f};
  cam_1_info.K = {stereo_imu.cam1_.cam_params_.fx(),
                  0.0f,
                  stereo_imu.cam1_.cam_params_.cx(),
                  0.0f,
                  stereo_imu.cam1_.cam_params_.fy(),
                  stereo_imu.cam1_.cam_params_.cy(),
                  0.0f,
                  0.0f,
                  1.0f};
  cam_1_info.R = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  Eigen::Matrix<double, 3, 4> T_c1_c0 = stereo_imu.T_c1_c0_.block<3, 4>(0, 0);
  Eigen::Matrix3d K_1;
  K_1 << cam_1_info.K[0], cam_1_info.K[1], cam_1_info.K[2], cam_1_info.K[3],
      cam_1_info.K[4], cam_1_info.K[5], cam_1_info.K[6], cam_1_info.K[7],
      cam_1_info.K[8];
  Eigen::Matrix<double, 3, 4> P_1 = K_1 * T_c1_c0;
  cam_1_info.P = {P_1(0, 0), P_1(0, 1), P_1(0, 2), P_1(0, 3),
                  P_1(1, 0), P_1(1, 1), P_1(1, 2), P_1(1, 3),
                  P_1(2, 0), P_1(2, 1), P_1(2, 2), P_1(2, 3)};

  // STEP: publish the tf on the topic, T_b_c0, T_b_c1, T_b_t
  ros::Publisher T_b_c0_pub =
      nh.advertise<geometry_msgs::TransformStamped>("T_b_c_0", 10);
  ros::Publisher T_b_c1_pub =
      nh.advertise<geometry_msgs::TransformStamped>("T_b_c_1", 10);
  ros::Publisher T_b_t_pub;
  if (has_tag) {
    T_b_t_pub = nh.advertise<geometry_msgs::TransformStamped>("T_b_t", 10);
  }

  ros::Rate publish_rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    publish_rate.sleep();
    ros::Time time_now = ros::Time::now();

    // pub camera info
    cam_0_info.header.stamp = time_now;
    cam_0_info_pub.publish(cam_0_info);

    cam_1_info.header.stamp = time_now;
    cam_1_info_pub.publish(cam_1_info);

    // publish the transform
    trans_b_c0.header.stamp = time_now;
    T_b_c0_pub.publish(trans_b_c0);
    trans_b_c1.header.stamp = time_now;
    T_b_c1_pub.publish(trans_b_c1);
    if (has_tag) {
      trans_b_t.header.stamp = time_now;
      T_b_t_pub.publish(trans_b_t);
    }
  }

  return 0;
}
