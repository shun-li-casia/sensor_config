/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: pinhole_camera.h
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

#ifndef SENSOR_CONFIG_PINHOLE_CAMERA_H
#define SENSOR_CONFIG_PINHOLE_CAMERA_H
#include "sensor_config/camera_model/camera.h"

namespace sensor_config {
class PinholeCamera : public Camera {
 public:
  class Parameters : public Camera::Parameters {
   public:
    Parameters();
    Parameters(const std::string& camera_name, int w, int h, double fx,
               double fy, double cx, double cy, double k1, double k2, double p1,
               double p2);

    double& fx();
    double& fy();
    double& cx();
    double& cy();
    double& k1();
    double& k2();
    double& p1();
    double& p2();

    double fx() const;
    double fy() const;
    double cx() const;
    double cy() const;
    double k1() const;
    double k2() const;
    double p1() const;
    double p2() const;

    Parameters& operator=(const Parameters& other);
    friend std::ostream& operator<<(std::ostream& out,
                                    const Parameters& params);

   private:
    double fx_{0.0f}, fy_{0.0f}, cx_{0.0f}, cy_{0.0f};
    double k1_{0.0f}, k2_{0.0f}, p1_{0.0f}, p2_{0.0f};
  };
};

}  // namespace sensor_config

#endif
