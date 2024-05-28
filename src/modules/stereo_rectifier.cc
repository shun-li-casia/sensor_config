/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: stereo_rectifier.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 28/09/2023
 *
 *   @Description:
 *
 *******************************************************************************/

#include "sensor_config/modules/stereo_rectifier.h"
#include "utility_tool/pcm_debug_helper.h"

namespace sensor_config {
void StereoRectifier::RectStereoParam(
    const Eigen::Matrix3d& Rrl, const Eigen::Vector3d& trl,
    Eigen::Matrix3d* rect_Rrl, Eigen::Vector3d* rect_trl,
    sensor_config::PinholeCamera::Parameters* cam_param_l,
    sensor_config::PinholeCamera::Parameters* cam_param_r,
    std::pair<cv::Mat, cv::Mat>* cam_l_maps,
    std::pair<cv::Mat, cv::Mat>* cam_r_maps) {
  cv::Mat Kl =
      (cv::Mat_<double>(3, 3) << cam_param_l->fx(), 0, cam_param_l->cx(), 0,
       cam_param_l->fy(), cam_param_l->cy(), 0, 0, 1);
  cv::Mat Dl = (cv::Mat_<double>(1, 4) << cam_param_l->k1(), cam_param_l->k2(),
                cam_param_l->p1(), cam_param_l->p2());

  cv::Mat Kr =
      (cv::Mat_<double>(3, 3) << cam_param_r->fx(), 0, cam_param_r->cx(), 0,
       cam_param_r->fy(), cam_param_r->cy(), 0, 0, 1);
  cv::Mat Dr = (cv::Mat_<double>(1, 4) << cam_param_r->k1(), cam_param_r->k2(),
                cam_param_r->p1(), cam_param_r->p2());

  cv::Size image_size(cam_param_l->img_w(), cam_param_l->img_h());

  cv::Mat cv_Rrl, cv_trl;
  cv::eigen2cv(Rrl, cv_Rrl);
  cv::eigen2cv(trl, cv_trl);

  cv::Mat Rl, Rr, Pl, Pr, Q;
  cv::stereoRectify(Kl, Dl, Kr, Dr, image_size, cv_Rrl, cv_trl, Rl, Rr, Pl, Pr,
                    Q, 0, -1);

  cv::Mat cv_rect_Rrl, cv_rect_trl;
  cv_rect_Rrl = Rr * cv_Rrl * Rl.t();
  cv_rect_trl = Rr * cv_trl;

  cv::cv2eigen(cv_rect_Rrl, *rect_Rrl);
  cv::cv2eigen(cv_rect_trl, *rect_trl);

  cv::Mat map11, map12;
  cv::initUndistortRectifyMap(Kl, Dl, Rl, Pl, image_size, CV_32F, map11, map12);
  *cam_l_maps = {map11, map12};

  cv::Mat map21, map22;
  cv::initUndistortRectifyMap(Kr, Dr, Rr, Pr, image_size, CV_32F, map21, map22);
  *cam_r_maps = {map21, map22};

  PCM_STREAM_VAR_DEBUG(Pl);
  PCM_STREAM_VAR_DEBUG(Pr);
  cam_param_l->k1() = 0;
  cam_param_l->k2() = 0;
  cam_param_l->p1() = 0;
  cam_param_l->p2() = 0;

  cam_param_l->fx() = Pl.at<double>(0, 0);
  cam_param_l->fy() = Pl.at<double>(1, 1);
  cam_param_l->cx() = Pl.at<double>(0, 2);
  cam_param_l->cy() = Pl.at<double>(1, 2);

  cam_param_r->k1() = 0;
  cam_param_r->k2() = 0;
  cam_param_r->p1() = 0;
  cam_param_r->p2() = 0;

  cam_param_r->fx() = Pr.at<double>(0, 0);
  cam_param_r->fy() = Pr.at<double>(1, 1);
  cam_param_r->cx() = Pr.at<double>(0, 2);
  cam_param_r->cy() = Pr.at<double>(1, 2);
}

}  // namespace sensor_config
