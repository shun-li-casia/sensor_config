/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: tp_checker.h
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 07/09/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef UTILITY_TOOL_TP_CHECKER_H_
#define UTILITY_TOOL_TP_CHECKER_H_

#include <utility_tool/print_ctrl_macro.h>
#include <utility_tool/pcm_debug_helper.h>
#include <utility_tool/file_writter.h>
#include <utility_tool/system_lib.h>
#include <ros/ros.h>

namespace sensor_config {
template <typename MessageType>
class TpChecker {
 public:
  typedef typename MessageType::ConstPtr MessageConstPtr;

  TpChecker(const std::string& topic, const std::string filename = "")
      : topic_(topic), filename_(filename) {
    sub_ = nh_.subscribe(topic_, 100, &TpChecker::CheckTp, this);
    last_tp_ = ros::Time(0);

    if (filename_.empty()) {
      filename_ = utility_tool::ReplaceEleInStr(topic_, "/", "_") + "_" +
                  utility_tool::GetCurLocalTimeStr("%Y%m%d%H%M%S") + ".csv";
    }

    file_writter_ = std::make_shared<utility_tool::FileWritter>(filename_, 6);
    file_writter_->SetDelimiter(",");
    file_writter_->EraseOpen();
    PCM_PRINT_DEBUG("Finish the construct!\n");
  }

  void Spin() {
    PCM_PRINT_INFO("Start to listen topic: %s\n", topic_.c_str());
    PCM_PRINT_INFO("Output file: %s\n", filename_.c_str());
    ros::spin();
  }

 private:
  std::string topic_;
  std::string filename_;
  utility_tool::FileWritter::Ptr file_writter_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  ros::Time last_tp_;

  void CheckTp(const MessageConstPtr& msg) {
    if (last_tp_ == ros::Time(0)) {
      last_tp_ = msg->header.stamp;
      PCM_PRINT_DEBUG("IN\n");
    } else {
      ros::Duration diff = msg->header.stamp - last_tp_;
      PCM_PRINT_INFO("current tp: %lf, diff: %lf\n", msg->header.stamp.toSec(),
                     diff.toSec());
      file_writter_->Write(msg->header.stamp.toSec(), diff.toSec());
      last_tp_ = msg->header.stamp;
    }
  }
};

}  // namespace sensor_config

#endif
