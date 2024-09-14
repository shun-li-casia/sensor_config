/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: open_imu.cc
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 14/09/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include "sensor_config/modules/imu_driver/serial_device.h"
#include "sensor_config/modules/imu_driver/imu_protocol.h"

#include "opencv2/imgproc.hpp"
#include "utility_tool/cmdline.h"
#include "utility_tool/file_writter.h"
#include "utility_tool/system_lib.h"
#include "utility_tool/print_ctrl_macro.h"
#include <cmath>

#include <ros/duration.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <atomic>

struct imu_data {
  float gyr_x_{0.0f};
  float gyr_y_{0.0f};
  float gyr_z_{0.0f};
  float acc_x_{0.0f};
  float acc_y_{0.0f};
  float acc_z_{0.0f};
  uint16_t stamp_{0};
  uint32_t frame_id_{0u};
};

constexpr float g = 9.8015;
constexpr float pi_div_180 = M_PI / 180.0f;
// t_imu = t_cam + time_shift
// NOTE: 49.02
constexpr double g_imu_t_step_table[] = {48.9578, 49.02, 49.02, 48.7803, 49.02};
std::atomic<double> g_imu_t_step_s;
std::atomic<int> g_imu_cnt;

std::atomic<bool> g_imu_is_ready;
static unsigned int uart_baudrate = 1500000;

ros::Publisher g_imu_pub;
ros::Time g_time_start, g_imu_time;
std::mutex g_imu_t_mutex;

uint16_t g_last_stamp = UINT16_MAX, g_max_stamp = 0;
bool g_is_first_frame = true;

uint32_t g_imu_seq = 0, g_img_seq = 0;
char* g_imu_env_value = NULL;
utility_tool::FileWritter::Ptr g_imu_writter;

void ImuCallback(unsigned char* data_block, int data_block_len) {
  if (!g_imu_is_ready.load()) {
    return;
  }
  struct imu_data data;

  if (g_imu_env_value != NULL) {
    if (*g_imu_env_value == '1') {
      data.gyr_x_ = (int16_t)(data_block[0] | (data_block[1] << 8)) * 1.0 *
                    0.00625 * pi_div_180;
      data.gyr_y_ = (int16_t)(data_block[2] | (data_block[3] << 8)) * 1.0 *
                    0.00625 * pi_div_180;
      data.gyr_z_ = (int16_t)(data_block[4] | (data_block[5] << 8)) * 1.0 *
                    0.00625 * pi_div_180;
    } else if (*g_imu_env_value == '2') {
      data.gyr_x_ = (int16_t)(data_block[0] | (data_block[1] << 8)) * 1.0 *
                    0.025 * pi_div_180;
      data.gyr_y_ = (int16_t)(data_block[2] | (data_block[3] << 8)) * 1.0 *
                    0.025 * pi_div_180;
      data.gyr_z_ = (int16_t)(data_block[4] | (data_block[5] << 8)) * 1.0 *
                    0.025 * pi_div_180;
    }
  } else {
    PCM_PRINT_ERROR("The environment variable IMU_ID is not set.\n");
    return;
  }

  data.acc_x_ =
      (int16_t)(data_block[6] | (data_block[7] << 8)) * 1.0 * 0.00025 * g;
  data.acc_y_ =
      (int16_t)(data_block[8] | (data_block[9] << 8)) * 1.0 * 0.00025 * g;
  data.acc_z_ =
      (int16_t)(data_block[10] | (data_block[11] << 8)) * 1.0 * 0.00025 * g;
  data.stamp_ = (uint16_t)(data_block[12] | (data_block[13] << 8));
  data.frame_id_ = (uint32_t)(data_block[18] | (data_block[19] << 8) |
                              (data_block[20] << 16) | (data_block[21] << 24));

  if (g_is_first_frame) {
    g_is_first_frame = false;
    g_time_start = ros::Time::now();
    g_imu_t_mutex.lock();
    g_imu_time = g_time_start;
    g_imu_t_mutex.unlock();
  } else {
    double time_diff_s = 0.0f;
    if (data.stamp_ <= 812) {
      // update the max stamp
      if (data.stamp_ > g_max_stamp) {
        g_max_stamp = data.stamp_;
      }

      if (data.stamp_ > g_last_stamp) {
        uint16_t stamp_diff = data.stamp_ - g_last_stamp;
        time_diff_s = stamp_diff * g_imu_t_step_s.load();
      } else if (data.stamp_ < g_last_stamp) {
        uint16_t stamp_diff = g_max_stamp - g_last_stamp + 10 + data.stamp_;
        time_diff_s = stamp_diff * g_imu_t_step_s.load();
      }

      // update the last stamp
      g_last_stamp = data.stamp_;
    } else {
      g_imu_t_mutex.lock();
      g_imu_writter->Write(g_imu_time, data.stamp_, time_diff_s, data.acc_x_,
                           data.acc_y_, data.acc_z_, data.gyr_x_, data.gyr_y_,
                           data.gyr_z_, "IMU_ERROR");
      g_imu_t_mutex.unlock();
      return;
    }

    g_imu_t_mutex.lock();
    g_imu_time += ros::Duration(time_diff_s);
    g_imu_writter->Write(g_imu_time, data.stamp_, time_diff_s);
    g_imu_t_mutex.unlock();

    g_imu_cnt.fetch_add(1);
  }

  sensor_msgs::Imu imu_msg;
  imu_msg.header.seq = data.frame_id_;
  imu_msg.header.stamp = g_imu_time;
  imu_msg.header.frame_id = "imu_raw_0";
  imu_msg.angular_velocity.x = data.gyr_x_;
  imu_msg.angular_velocity.y = data.gyr_y_;
  imu_msg.angular_velocity.z = data.gyr_z_;
  imu_msg.linear_acceleration.x = data.acc_x_;
  imu_msg.linear_acceleration.y = data.acc_y_;
  imu_msg.linear_acceleration.z = data.acc_z_;

  g_imu_pub.publish(imu_msg);
}

int main(int argc, char* argv[]) {
  cmdline::parser par;

  par.add<int>("uav_id", 'u', "uav id", true, 0);
  par.add<int>("imu_uart", 'i', "imu_uart id", true, 0);
  par.add<float>("imu_step", 's', "imu step in us", false);
  par.parse_check(argc, argv);

  int uav_id = par.get<int>("uav_id");
  if (uav_id < 0 || uav_id > 4) {
    PCM_PRINT_ERROR("uav id should be in [0, 4]\n");
    return -1;
  }
  g_imu_t_step_s.store(g_imu_t_step_table[uav_id] * 1e-6);
  if (par.exist("imu_step")) {
    g_imu_t_step_s.store(par.get<float>("imu_step") * 1e-6);
  }

  ros::init(argc, argv, "open_imu_to_rosmsg_node");
  ros::NodeHandle nh;

  // init imu
  g_imu_env_value = getenv("IMU_ID");
  g_imu_is_ready.store(false);
  g_imu_writter = std::make_shared<utility_tool::FileWritter>(
      "imu_debug_" + utility_tool::GetCurLocalTimeStr("%Y%m%d%H%M%S") + ".csv",
      6);
  g_imu_writter->SetDelimiter(",");
  g_imu_writter->EraseOpen();

  std::string uart1 = "/dev/ttyUSB" + std::to_string(par.get<int>("imu_uart"));
  if (Set_Serial_Parse_Callback(ImuCallback) < 0) {
    PCM_PRINT_ERROR("set imu callback failed!\n");
    return -1;
  } else {
    PCM_PRINT_INFO("set imu callback successfully!\n");
  }

  if (Serial_Device_Init(uart1.c_str(), uart_baudrate) < 0) {
    PCM_PRINT_ERROR("open %s failed!\n", uart1.c_str());
    return -1;
  } else {
    PCM_PRINT_INFO("open %s successfully!\n", uart1.c_str());
  }

  // open the imu
  if (get_imu_data_start() < 0) {
    PCM_PRINT_ERROR("can not open the serial %s", uart1.c_str());
    return -1;
  }
  g_imu_is_ready.store(true);
  g_time_start = ros::Time::now();
  g_imu_time = g_time_start;
  PCM_PRINT_INFO("start IMU!\n");

  Serial_Device_UnInit();
  PCM_PRINT_INFO("the node is shutdown!\n");
  return 0;
}
