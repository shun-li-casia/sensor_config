/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: stereo_imu_config.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 26/06/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "sensor_config/modules/stereo_imu_config.h"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "utility_tool/print_ctrl_macro.h"

namespace sensor_config {

bool StereoImu::readKalibr(const std::string& cam_imu_chain_path,
                           const std::string& imu_path) {
  // read imu0 info
  if (!imu0_.readFromKalibr(imu_path)) {
    return false;
  };

  // read cam0 and cam1
  cam0_.cam_params_.camera_name() = "cam0";
  if (!cam0_.cam_params_.readKalibrSingleCam(cam_imu_chain_path)) {
    return false;
  }
  cam0_.cam_params_.camera_name() = "uav_" + std::to_string(uav_id_) + "_cam0";

  cam1_.cam_params_.camera_name() = "cam1";
  if (!cam1_.cam_params_.readKalibrSingleCam(cam_imu_chain_path)) {
    return false;
  }
  cam1_.cam_params_.camera_name() = "uav_" + std::to_string(uav_id_) + "_cam1";

  auto n = YAML::LoadFile(cam_imu_chain_path);
  if (n.IsNull()) {
    return false;
  }

  cam0_.time_shift_ = n["cam0"]["timeshift_cam_imu"].as<double>();
  cam1_.time_shift_ = n["cam1"]["timeshift_cam_imu"].as<double>();

  std::vector<std::vector<double>> raw_T_c0_i =
      n["cam0"]["T_cam_imu"].as<std::vector<std::vector<double>>>();
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      cam0_.Tci_(x, y) = raw_T_c0_i[x][y];
    }
  }

  std::vector<std::vector<double>> raw_T_c1_i =
      n["cam1"]["T_cam_imu"].as<std::vector<std::vector<double>>>();
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      cam1_.Tci_(x, y) = raw_T_c1_i[x][y];
    }
  }

  // for cam0-cam1
  std::vector<std::vector<double>> raw_T_cn_cnm1 =
      n["cam1"]["T_cn_cnm1"].as<std::vector<std::vector<double>>>();
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      T_c1_c0_(x, y) = raw_T_cn_cnm1[x][y];
    }
  }

  return true;
}

void StereoImu::writeCameraCV(const CamInStereo& cam) const {
  // 创建一个YAML文档
  YAML::Node root;

  // 添加数据
  root["model_type"] = "PINHOLE";
  root["camera_name"] = cam.cam_params_.camera_name();
  root["image_width"] = cam.cam_params_.img_w();
  root["image_height"] = cam.cam_params_.img_h();

  // 畸变参数
  YAML::Node distortion_parameters;
  distortion_parameters["k1"] = cam.cam_params_.k1();
  distortion_parameters["k2"] = cam.cam_params_.k2();
  distortion_parameters["p1"] = cam.cam_params_.p1();
  distortion_parameters["p2"] = cam.cam_params_.p2();
  root["distortion_parameters"] = distortion_parameters;

  // 投影参数
  YAML::Node projection_parameters;
  projection_parameters["fx"] = cam.cam_params_.fx();
  projection_parameters["fy"] = cam.cam_params_.fy();
  projection_parameters["cx"] = cam.cam_params_.cx();
  projection_parameters["cy"] = cam.cam_params_.cy();
  root["projection_parameters"] = projection_parameters;

  // 将数据写入文件
  std::ofstream fout(cam.cam_params_.camera_name() + ".yaml");
  fout << "%YAML:1.0" << std::endl;
  fout << "---" << std::endl;
  fout << root;
  fout.close();
}

bool StereoImu::writeVins(const std::string& path, const int uav_id) const {
  // wirte the camera file
  writeCameraCV(cam0_);
  writeCameraCV(cam1_);

  // write the main file
  //  使用cv::FileStorage来保存数据
  cv::FileStorage fs(path, cv::FileStorage::WRITE);

  // 写入基本的整数和字符串数据
  // fs << "%YAML:1.0";
  fs << "uav_id" << uav_id;
  fs << "imu" << 1;
  fs << "num_of_cam" << 2;
  fs << "imu_topic" << "/uav_" + std::to_string(uav_id_) + "/imu_raw_0";
  fs << "image0_topic" << "/uav_" + std::to_string(uav_id_) + "/cam_0";
  fs << "image1_topic" << "/uav_" + std::to_string(uav_id_) + "/cam_1";
  fs << "output_path" << "~/vins_output";
  fs << "cam0_calib" << cam0_.cam_params_.camera_name() + ".yaml";
  fs << "cam1_calib" << cam1_.cam_params_.camera_name() + ".yaml";
  fs << "image_width" << cam0_.cam_params_.img_w();
  fs << "image_height" << cam0_.cam_params_.img_h();

  // 创建IMU到相机的变换矩阵
  cv::Mat body_T_cam0(4, 4, CV_64F);
  Sophus::SE3d Tc0i(cam0_.Tci_);
  Eigen::Matrix4d eigen_Tic0 = Tc0i.inverse().matrix();
  cv::eigen2cv(eigen_Tic0, body_T_cam0);

  cv::Mat body_T_cam1(4, 4, CV_64F);
  Sophus::SE3d Tc1i(cam1_.Tci_);
  Eigen::Matrix4d eigen_Tic1 = Tc1i.inverse().matrix();
  cv::eigen2cv(eigen_Tic1, body_T_cam1);

  // 保存矩阵数据
  fs << "body_T_cam0" << body_T_cam0;
  fs << "body_T_cam1" << body_T_cam1;

  // Write other parameters
  fs << "multiple_thread" << 1;
  fs << "max_cnt" << 500;  // default 200
  fs << "min_dist" << 10;  // default 10
  fs << "freq" << 25;
  fs << "F_threshold" << 1.0;
  fs << "show_track" << 1;
  fs << "flow_back" << 1;
  fs << "max_solver_time" << 0.04;
  fs << "max_num_iterations" << 10;  // default 10
  fs << "keyframe_parallax" << 10.0;

  // NOTE: 10 times bigger of the noise of the calibrated imu
  fs << "acc_n" << imu0_.acc_n() * 10.0f;
  fs << "gyr_n" << imu0_.gyr_n() * 10.0f;
  fs << "acc_w" << imu0_.acc_w() * 10.0f;
  fs << "gyr_w" << imu0_.gyr_w() * 10.0f;

  fs << "g_norm" << 9.81007;
  fs << "estimate_td" << 0;  // default 1
  fs << "td" << (cam0_.time_shift_ + cam1_.time_shift_) * 0.5f;
  fs << "load_previous_pose_graph" << 0;
  fs << "pose_graph_save_path" << "~/output/pose_graph/";
  fs << "save_image" << 0;

  // 关闭文件存储
  fs.release();

  PCM_STREAM_INFO("YAML file has been saved.");

  return true;
}
}  // namespace sensor_config
