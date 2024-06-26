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
#include <opencv2/opencv.hpp>

namespace sensor_config {
class Camera {
  typedef std::shared_ptr<Camera> Ptr;

 public:
  enum class CameraModel {
    PINHOLE = 0,
    PINHOLE_FULL,
  };

  enum class DistortionModel { RADIAL_TANGENTIAL = 0 };

  // FIXME: the rostopic and overlaps!
  class Parameters {
   public:
    Parameters(CameraModel camera_model, DistortionModel distortion_model);
    Parameters(CameraModel camera_model, DistortionModel distortion_model,
               std::string camera_name, int img_w, int img_h);

    CameraModel& camera_model();
    DistortionModel& distortion_model();
    std::string& camera_name();
    std::string& rostopic();
    int& img_w();
    int& img_h();
    std::vector<int>& cam_overlaps();

    CameraModel camera_model() const;
    DistortionModel distortion_model() const;
    const std::string& camera_name() const;
    const std::string& rostopic() const;
    int img_w() const;
    int img_h() const;
    std::vector<int> cam_overlaps() const;

    virtual bool readKalibrSingleCam(const std::string& path) = 0;
    virtual void writeKalibrSingleCam(const std::string& path) const = 0;

   protected:
    CameraModel camera_model_;
    DistortionModel distortion_model_;
    std::string camera_name_;
    int img_w_{0}, img_h_{0};

    // for kalibr
    std::vector<int> cam_overlaps_;
    std::string rostopic_;
  };
};
}  // namespace sensor_config

#endif
