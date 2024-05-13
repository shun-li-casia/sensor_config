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
Camera::Parameters::Parameters(ModelType model_type)
    : model_type_(model_type), img_w_(0), img_h_(0) {}

Camera::Parameters::Parameters(ModelType model_type, std::string camera_name,
                               int img_w, int img_h)
    : model_type_(model_type),
      camera_name_(camera_name),
      img_w_(img_w),
      img_h_(img_h) {}

Camera::ModelType& Camera::Parameters::model_type() { return model_type_; }

std::string& Camera::Parameters::camera_name() { return camera_name_; }

int& Camera::Parameters::img_w() { return img_w_; }

int& Camera::Parameters::img_h() { return img_h_; }

Camera::ModelType Camera::Parameters::modelType() const { return model_type_; }

const std::string& Camera::Parameters::camera_name() const {
  return camera_name_;
}

int Camera::Parameters::img_w() const { return img_w_; }

int Camera::Parameters::img_h() const { return img_h_; }
}  // namespace sensor_config
