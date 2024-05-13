/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: camera.h
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

#ifndef SENSOR_CONFIG_CAMERA_H
#define SENSOR_CONFIG_CAMERA_H

#include <string>
#include <memory>

namespace sensor_config {
class Camera {
  typedef std::shared_ptr<Camera> Ptr;

 public:
  enum class ModelType {
    PINHOLE = 0,
    PINHOLE_FULL,
  };

  class Parameters {
   public:
    Parameters(ModelType model_type);
    Parameters(ModelType model_type, std::string camera_name, int img_w,
               int img_h);

    ModelType& model_type();
    std::string& camera_name();
    int& img_w();
    int& img_h();

    ModelType modelType() const;
    const std::string& camera_name() const;
    int img_w() const;
    int img_h() const;

    virtual Parameters& operator=(const Parameters& other) = 0;

   protected:
    ModelType model_type_;
    std::string camera_name_;
    int img_w_{0}, img_h_{0};
  };
};
}  // namespace sensor_config

#endif
