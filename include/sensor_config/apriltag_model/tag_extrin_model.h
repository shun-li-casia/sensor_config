/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: tag_extrin_model.h
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 06/08/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef SENSOR_CONFIG_TAG_EXTRIN_MODEL_H
#define SENSOR_CONFIG_TAG_EXTRIN_MODEL_H

#include "utility_tool/print_ctrl_macro.h"
#include <geometry_msgs/TransformStamped.h>
#include <yaml-cpp/yaml.h>

class TagExtrinModel {
 public:
  geometry_msgs::TransformStamped T_b_t_;

  bool readTagExtrin(const std::string& path) {
    auto n = YAML::LoadFile(path);
    if (n.IsNull()) {
      PCM_PRINT_ERROR("load tag extrin failed: %s", path.c_str());
      return false;
    }

    T_b_t_.header.frame_id = n["header"]["frame_id"].as<std::string>();
    T_b_t_.child_frame_id = n["child_frame_id"].as<std::string>();
    T_b_t_.transform.translation.x =
        n["transform"]["translation"][0].as<double>();
    T_b_t_.transform.translation.y =
        n["transform"]["translation"][1].as<double>();
    T_b_t_.transform.translation.z =
        n["transform"]["translation"][2].as<double>();
    T_b_t_.transform.rotation.x = n["transform"]["rotation"][0].as<double>();
    T_b_t_.transform.rotation.y = n["transform"]["rotation"][1].as<double>();
    T_b_t_.transform.rotation.z = n["transform"]["rotation"][2].as<double>();
    T_b_t_.transform.rotation.w = n["transform"]["rotation"][3].as<double>();

    return true;
  }
};

#endif
