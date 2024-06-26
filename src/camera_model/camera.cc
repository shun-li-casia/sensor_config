/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: camera.cc
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

#include "sensor_config/camera_model/camera.h"

namespace sensor_config {
Camera::Parameters::Parameters(CameraModel camera_model,
                               DistortionModel distortion_model)
    : camera_model_(camera_model),
      distortion_model_(distortion_model),
      img_w_(0),
      img_h_(0) {}

Camera::Parameters::Parameters(CameraModel camera_model,
                               DistortionModel distortion_model,
                               std::string camera_name, int img_w, int img_h)
    : camera_model_(camera_model),
      distortion_model_(distortion_model),
      camera_name_(camera_name),
      img_w_(img_w),
      img_h_(img_h) {}

Camera::CameraModel& Camera::Parameters::camera_model() {
  return camera_model_;
}
Camera::DistortionModel& Camera::Parameters::distortion_model() {
  return distortion_model_;
}

std::string& Camera::Parameters::camera_name() { return camera_name_; }

std::string& Camera::Parameters::rostopic() { return rostopic_; }

int& Camera::Parameters::img_w() { return img_w_; }

int& Camera::Parameters::img_h() { return img_h_; }

std::vector<int>& Camera::Parameters::cam_overlaps() { return cam_overlaps_; }

Camera::CameraModel Camera::Parameters::camera_model() const {
  return camera_model_;
}
Camera::DistortionModel Camera::Parameters::distortion_model() const {
  return distortion_model_;
}

const std::string& Camera::Parameters::camera_name() const {
  return camera_name_;
}

const std::string& Camera::Parameters::rostopic() const { return rostopic_; }

int Camera::Parameters::img_w() const { return img_w_; }

int Camera::Parameters::img_h() const { return img_h_; }

std::vector<int> Camera::Parameters::cam_overlaps() const {
  return cam_overlaps_;
}

}  // namespace sensor_config
