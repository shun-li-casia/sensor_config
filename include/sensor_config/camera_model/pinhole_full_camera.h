/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: pinhole_full_camera.h
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

#ifndef SENSOR_CONFIG_PINHOLE_FULL_CAMERA_H
#define SENSOR_CONFIG_PINHOLE_FULL_CAMERA_H
#include "sensor_config/camera_model/camera.h"

namespace sensor_config {
class PinholeFullCamera : public Camera {
 public:
  class Parameters : public Camera::Parameters {};
};

}  // namespace sensor_config

#endif
