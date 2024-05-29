/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: pinhole_camera.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 13/05/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "sensor_config/camera_model/pinhole_camera.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

namespace sensor_config {

PinholeCamera::Parameters::Parameters()
    : Camera::Parameters(Camera::ModelType::PINHOLE) {}

PinholeCamera::Parameters::Parameters(const std::string& camera_name, int img_w,
                                      int img_h, double fx, double fy,
                                      double cx, double cy, double k1,
                                      double k2, double p1, double p2)
    : Camera::Parameters(ModelType::PINHOLE, camera_name, img_w, img_h),
      fx_(fx),
      fy_(fy),
      cx_(cx),
      cy_(cy),
      k1_(k1),
      k2_(k2),
      p1_(p1),
      p2_(p2) {}

double& PinholeCamera::Parameters::fx() { return fx_; }
double& PinholeCamera::Parameters::fy() { return fy_; }
double& PinholeCamera::Parameters::cx() { return cx_; }
double& PinholeCamera::Parameters::cy() { return cy_; }
double& PinholeCamera::Parameters::k1() { return k1_; }
double& PinholeCamera::Parameters::k2() { return k2_; }
double& PinholeCamera::Parameters::p1() { return p1_; }
double& PinholeCamera::Parameters::p2() { return p2_; }

double PinholeCamera::Parameters::fx() const { return fx_; }
double PinholeCamera::Parameters::fy() const { return fy_; }
double PinholeCamera::Parameters::cx() const { return cx_; }
double PinholeCamera::Parameters::cy() const { return cy_; }
double PinholeCamera::Parameters::k1() const { return k1_; }
double PinholeCamera::Parameters::k2() const { return k2_; }
double PinholeCamera::Parameters::p1() const { return p1_; }
double PinholeCamera::Parameters::p2() const { return p2_; }

bool PinholeCamera::Parameters::readKalibrSingleCam(const std::string& path) {
  auto n = YAML::LoadFile(path);
  if (n.IsNull()) {
    return false;
  }

  std::string cam = "cam0";
  // intrinsics
  camera_name_ = cam;
  model_type_ = Camera::ModelType::PINHOLE;
  std::vector<int> resolution = n[cam]["resolution"].as<std::vector<int>>();
  img_w_ = resolution[0];
  img_h_ = resolution[1];

  std::vector<double> distortion_coeffs =
      n[cam]["distortion_coeffs"].as<std::vector<double>>();
  k1_ = distortion_coeffs[0];
  k2_ = distortion_coeffs[1];
  p1_ = distortion_coeffs[2];
  p2_ = distortion_coeffs[3];

  std::vector<double> intrinsics =
      n[cam]["intrinsics"].as<std::vector<double>>();
  fx_ = intrinsics[0];
  fy_ = intrinsics[1];
  cx_ = intrinsics[2];
  cy_ = intrinsics[3];

  std::vector<int> overlap = n[cam]["cam_overlaps"].as<std::vector<int>>();
  cam_overlaps_ = overlap;

  std::string ros_topic = n[cam]["rostopic"].as<std::string>();
  rostopic_ = ros_topic;
  return true;
}

void PinholeCamera::Parameters::writeKalibrSingleCam(
    const std::string& path) const {
  YAML::Node node;
  assert(node.IsNull());
  std::string cam_names{"cam1"};

  // overlap
  node[cam_names]["cam_overlaps"] = cam_overlaps_;
  node[cam_names]["cam_overlaps"].SetStyle(YAML::EmitterStyle::Flow);

  // camera model
  node[cam_names]["camera_model"] = "pinhole";

  // distortion_coeffs
  std::vector<double> dis = {k1_, k2_, p1_, p2_};
  node[cam_names]["distortion_coeffs"] = dis;
  node[cam_names]["distortion_coeffs"].SetStyle(YAML::EmitterStyle::Flow);

  // distortion_model
  node[cam_names]["distortion_model"] = "radtan";

  // intrinsics
  std::vector<double> in = {fx_, fy_, cx_, cy_};
  node[cam_names]["intrinsics"] = in;
  node[cam_names]["intrinsics"].SetStyle(YAML::EmitterStyle::Flow);

  // resolution
  std::vector<int> resolution = {img_w_, img_h_};
  node[cam_names]["resolution"] = resolution;
  node[cam_names]["resolution"].SetStyle(YAML::EmitterStyle::Flow);

  // rostopic
  node[cam_names]["rostopic"] = rostopic_;

  std::ofstream file(path);
  file.clear();
  file << node;
  file.close();
}

PinholeCamera::Parameters& PinholeCamera::Parameters::operator=(
    const PinholeCamera::Parameters& other) {
  if (this == &other) {
    return *this;
  }

  model_type_ = other.model_type_;
  camera_name_ = other.camera_name_;
  img_w_ = other.img_w_;
  img_h_ = other.img_h_;
  fx_ = other.fx_;
  fy_ = other.fy_;
  cx_ = other.cx_;
  cy_ = other.cy_;
  k1_ = other.k1_;
  k2_ = other.k2_;
  p1_ = other.p1_;
  p2_ = other.p2_;
  return *this;
}

std::ostream& operator<<(std::ostream& out,
                         const PinholeCamera::Parameters& params) {
  out << "Camera Parameters:" << std::endl;
  out << "    model_type "
      << "PINHOLE" << std::endl;
  out << "   camera_name " << params.camera_name_ << std::endl;
  out << "   image_width " << params.img_w_ << std::endl;
  out << "  image_height " << params.img_h_ << std::endl;

  // radial distortion: k1, k2
  // tangential distortion: p1, p2
  out << "Distortion Parameters" << std::endl;
  out << "            k1 " << params.k1_ << std::endl
      << "            k2 " << params.k2_ << std::endl
      << "            p1 " << params.p1_ << std::endl
      << "            p2 " << params.p2_ << std::endl;

  // projection: fx, fy, cx, cy
  out << "Projection Parameters" << std::endl;
  out << "            fx " << params.fx_ << std::endl
      << "            fy " << params.fy_ << std::endl
      << "            cx " << params.cx_ << std::endl
      << "            cy " << params.cy_ << std::endl;

  return out;
}

PinholeCamera::PinholeCamera(const std::string& cameraName, int imageWidth,
                             int imageHeight, double fx, double fy, double cx,
                             double cy, double k1, double k2, double p1,
                             double p2)
    : params_(cameraName, imageWidth, imageHeight, fx, fy, cx, cy, k1, k2, p1,
              p2) {}

PinholeCamera::PinholeCamera(const PinholeCamera::Parameters& params)
    : params_(params) {}

void PinholeCamera::RectSignalCamParam(
    sensor_config::PinholeCamera::Parameters* undis_params,
    std::pair<cv::Mat, cv::Mat>* maps) {
  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << params_.fx(), 0,
                          params_.cx(), 0, params_.fy(), params_.cy(), 0, 0, 1);
  cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << params_.k1(), params_.k2(),
                        params_.p1(), params_.p2());

  cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix,
                              cv::Size(params_.img_w(), params_.img_h()),
                              CV_32FC1, maps->first, maps->second);

  undis_params->camera_name() = params_.camera_name() + "_rect";
  undis_params->img_w() = params_.img_w();
  undis_params->img_h() = params_.img_h();
  undis_params->fx() = cameraMatrix.at<double>(0, 0);
  undis_params->fy() = cameraMatrix.at<double>(1, 1);
  undis_params->cx() = cameraMatrix.at<double>(0, 2);
  undis_params->cy() = cameraMatrix.at<double>(1, 2);
  undis_params->k1() = 0;
  undis_params->k2() = 0;
  undis_params->p1() = 0;
  undis_params->p2() = 0;

  undis_params->rostopic() = params_.rostopic() + "_rect";
  undis_params->cam_overlaps() = params_.cam_overlaps();
}
}  // namespace sensor_config
