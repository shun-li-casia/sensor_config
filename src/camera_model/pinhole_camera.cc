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
}  // namespace sensor_config
