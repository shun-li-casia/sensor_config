/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: camera_models_kalibr.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 09/10/2023
 *
 *   @Description:
 *
 *******************************************************************************/

#include "sensor_config/camera_models_kalibr.h"
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace sensor_config {

void ConfigManager::ReadKalibr(const std::string calibn_file,
                               ImgImuConfig* conf, float resize_scale_factor) {
  conf->rostopic_.resize(2);
  conf->cam_params_.resize(2);
  conf->cam_overlaps_.resize(2);

  auto n = YAML::LoadFile(calibn_file);
  std::vector<std::string> cam_names{"cam0", "cam1"};

  for (int i = 0; i < 2; ++i) {
    // extrinsic
    auto cam = cam_names[i];

    if (i == 1) {  // cam 1
      std::vector<std::vector<double>> raw_T =
          n[cam]["T_cn_cnm1"].as<std::vector<std::vector<double>>>();
      Eigen::Matrix4d T21;
      for (int x = 0; x < 4; ++x) {
        for (int y = 0; y < 4; ++y) {
          T21(x, y) = raw_T[x][y];
        }
      }
      conf->r_rl_ = T21.block<3, 3>(0, 0);
      conf->t_rl_ = T21.block<3, 1>(0, 3);
    }

    // intrinsics
    conf->cam_params_[i].camera_name() = cam;
    conf->cam_params_[i].model_type() = Camera::ModelType::PINHOLE;
    std::vector<int> resolution = n[cam]["resolution"].as<std::vector<int>>();
    conf->cam_params_[i].img_w() = resolution[0] * resize_scale_factor;
    conf->cam_params_[i].img_h() = resolution[1] * resize_scale_factor;

    std::vector<double> distortion_coeffs =
        n[cam]["distortion_coeffs"].as<std::vector<double>>();
    conf->cam_params_[i].k1() = distortion_coeffs[0];
    conf->cam_params_[i].k2() = distortion_coeffs[1];
    conf->cam_params_[i].p1() = distortion_coeffs[2];
    conf->cam_params_[i].p2() = distortion_coeffs[3];

    std::vector<double> intrinsics =
        n[cam]["intrinsics"].as<std::vector<double>>();
    conf->cam_params_[i].fx() = intrinsics[0] * resize_scale_factor;
    conf->cam_params_[i].fy() = intrinsics[1] * resize_scale_factor;
    conf->cam_params_[i].cx() = intrinsics[2] * resize_scale_factor;
    conf->cam_params_[i].cy() = intrinsics[3] * resize_scale_factor;

    std::vector<int> overlap = n[cam]["cam_overlaps"].as<std::vector<int>>();
    conf->cam_overlaps_[i] = overlap;

    std::string ros_topic = n[cam]["rostopic"].as<std::string>();
    conf->rostopic_[i] = ros_topic;
  }
}

void ConfigManager::WriteKalibr(const ImgImuConfig& conf,

                                const std::string file_name) {
  YAML::Node node;
  assert(node.IsNull());
  std::vector<std::string> cam_names{"cam0", "cam1"};

  for (int i = 0; i < 2; ++i) {
    if (i == 1) {
      Eigen::Matrix<double, 3, 4> RT;
      RT << conf.r_rl_, conf.t_rl_;
      for (int j = 0; j < 3; ++j) {
        std::vector<double> row;
        row.resize(4);
        Eigen::Vector4d::Map(&row[0], 4) = RT.row(j);
        node[cam_names[i]]["T_cn_cnm1"].push_back(row);
        node[cam_names[i]]["T_cn_cnm1"][j].SetStyle(YAML::EmitterStyle::Flow);
      }

      std::vector<double> last_row = {0.0, 0.0, 0.0, 1.0};
      node[cam_names[i]]["T_cn_cnm1"].push_back(last_row);
      node[cam_names[i]]["T_cn_cnm1"][3].SetStyle(YAML::EmitterStyle::Flow);

      node[cam_names[i]]["T_cn_cnm1"].SetStyle(YAML::EmitterStyle::Block);
    }

    // overlap
    node[cam_names[i]]["cam_overlaps"] = conf.cam_overlaps_[i];
    node[cam_names[i]]["cam_overlaps"].SetStyle(YAML::EmitterStyle::Flow);

    // camera model
    node[cam_names[i]]["camera_model"] = "pinhole";

    // distortion_coeffs
    const PinholeCamera::Parameters& p = conf.cam_params_[i];
    std::vector<double> dis = {p.k1(), p.k2(), p.p1(), p.p2()};
    node[cam_names[i]]["distortion_coeffs"] = dis;
    node[cam_names[i]]["distortion_coeffs"].SetStyle(YAML::EmitterStyle::Flow);

    // distortion_model
    node[cam_names[i]]["distortion_model"] = "radtan";

    // intrinsics
    std::vector<double> in = {p.fx(), p.fy(), p.cx(), p.cy()};
    node[cam_names[i]]["intrinsics"] = in;
    node[cam_names[i]]["intrinsics"].SetStyle(YAML::EmitterStyle::Flow);

    // resolution
    std::vector<int> resolution = {p.img_w(), p.img_h()};
    node[cam_names[i]]["resolution"] = resolution;
    node[cam_names[i]]["resolution"].SetStyle(YAML::EmitterStyle::Flow);

    // rostopic
    node[cam_names[i]]["rostopic"] = conf.rostopic_[i];
  }

  std::ofstream file(file_name);
  file.clear();
  file << node;
  file.close();
}
}  // namespace sensor_config
