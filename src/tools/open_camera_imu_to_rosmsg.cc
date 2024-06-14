/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: open_camera_imu_to_rosmsg.cc
 *
 *   @Author: Ting Cai, Shun Li
 *
 *
 *   @Date: 13/06/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include "sensor_config/imu_driver/serial_device.h"
#include "sensor_config/imu_driver/imu_protocol.h"

#include "opencv2/imgproc.hpp"
#include "utility_tool/cmdline.h"
#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/pcm_debug_helper.h"
#include "utility_tool/system_lib.h"

#include <ros/duration.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
#include <mutex>

struct imu_data {
  uint32_t tp_{0u};
  uint32_t frame_id_{0u};
  float gyr_x_{0.0f};
  float gyr_y_{0.0f};
  float gyr_z_{0.0f};
  float acc_x_{0.0f};
  float acc_y_{0.0f};
  float acc_z_{0.0f};
};

static bool g_imu_is_ready = false;
static uint32_t g_imu_cnt = 0;
static unsigned int uart_baudrate = 1500000;

ros::Publisher g_imu_pub;
ros::Time g_time_start, g_imu_time;
std::mutex g_imu_t_mutex;

uint32_t g_last_tp_ = UINT32_MAX;
imu_data sum_data;
uint32_t average_cnt = 0;
bool is_first_frame = true;

void ImuCallback(unsigned char* data_block, int data_block_len) {
  if (!g_imu_is_ready) {
    ros::Duration(0, 1e6);
    return;
  }

  struct imu_data data;

  data.gyr_x_ = (int16_t)(data_block[0] | (data_block[1] << 8)) * 1.0 * 0.025;
  data.gyr_y_ = (int16_t)(data_block[2] | (data_block[3] << 8)) * 1.0 * 0.025;
  data.gyr_z_ = (int16_t)(data_block[4] | (data_block[5] << 8)) * 1.0 * 0.025;
  data.acc_x_ = (int16_t)(data_block[6] | (data_block[7] << 8)) * 1.0 * 0.00025;
  data.acc_y_ = (int16_t)(data_block[8] | (data_block[9] << 8)) * 1.0 * 0.00025;
  data.acc_z_ =
      (int16_t)(data_block[10] | (data_block[11] << 8)) * 1.0 * 0.00025;
  data.tp_ = (uint32_t)(data_block[12] | (data_block[13] << 8) |
                        (data_block[14] << 16) | (data_block[15] << 24));
  data.frame_id_ = (uint32_t)(data_block[16] | (data_block[17] << 8) |
                              (data_block[18] << 16) | (data_block[19] << 24));

  if (is_first_frame) {
    g_last_tp_ = data.tp_;
    is_first_frame = false;
  }

  if (data.tp_ != g_last_tp_) {
    // ready to pub
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "body";
    ros::Duration offset(g_last_tp_ * 1e-3f);
    imu_msg.header.stamp = g_time_start + offset;

    if (g_imu_t_mutex.try_lock()) {
      g_imu_time = g_time_start + ros::Duration(data.tp_ * 1e-3f);
      g_imu_t_mutex.unlock();
    }

    imu_msg.angular_velocity.x = sum_data.gyr_x_ / average_cnt;
    imu_msg.angular_velocity.y = sum_data.gyr_y_ / average_cnt;
    imu_msg.angular_velocity.z = sum_data.gyr_z_ / average_cnt;
    imu_msg.linear_acceleration.x = sum_data.acc_x_ / average_cnt;
    imu_msg.linear_acceleration.y = sum_data.acc_y_ / average_cnt;
    imu_msg.linear_acceleration.z = sum_data.acc_z_ / average_cnt;

    g_imu_pub.publish(imu_msg);
    printf("%u,%zu\n", data.tp_, offset.toNSec() / 1000);

    // reset the data
    average_cnt = 0;
    sum_data = imu_data();
  }

  sum_data.gyr_x_ += data.gyr_x_;
  sum_data.gyr_y_ += data.gyr_y_;
  sum_data.gyr_z_ += data.gyr_z_;
  sum_data.acc_x_ += data.acc_x_;
  sum_data.acc_y_ += data.acc_y_;
  sum_data.acc_z_ += data.acc_z_;
  average_cnt++;

  g_last_tp_ = data.tp_;
  g_imu_cnt++;
}

int main(int argc, char* argv[]) {
  cmdline::parser par;

  par.add<int>("imu_uart", 'i', "imu_uart id", true, 0);
  par.add<int>("camera_id", 'c', "camera id", true, 0);
  par.parse_check(argc, argv);

  ros::init(argc, argv, "open_camera_imu_to_rosmsg_node");
  ros::NodeHandle nh;

  // init imu
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

  int camera_id = par.get<int>("camera_id");
  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>(
      "image_raw_" + std::to_string(camera_id), 1);
  g_imu_pub = nh.advertise<sensor_msgs::Imu>("imu_raw_0", 10000);

  cv::VideoCapture cap;

  // loop try to open the camera
  for (int i = 0; i < 10; ++i) {
    cap.open(camera_id, cv::CAP_V4L2);
    bool is_open = cap.isOpened();
    if (is_open) {
      PCM_PRINT_INFO("open the camera successfully!\n");
      break;
    }
    if (!is_open && i == 9) {
      PCM_PRINT_ERROR("can not open the camera after %d times! Exit!\n", i + 1);
      return -1;
    }

    PCM_PRINT_INFO("tryed %d times to open the camera.\n", i + 1);
    ros::Duration(1).sleep();
  }

  cap.set(cv::CAP_PROP_CONVERT_RGB, 0);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 3840);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
  cap.set(cv::CAP_PROP_FPS, 25);

  if (!cap.isOpened()) {
    PCM_PRINT_ERROR("can not open the camera!\n");
    return -1;
  } else {
    PCM_PRINT_INFO("open the camera successfully!\n");
  }

  utility_tool::Timer timer, total;
  total.Start();

  const int camera_is_stable = 10;
  int count = 0;

  while (ros::ok()) {
    timer.Start();
    cv::Mat frame, rgb;
    cap >> frame;
    if (frame.empty()) {
      PCM_PRINT_WARN("frame is empty!\n");
      continue;
    }

    if (count++ == camera_is_stable) {
      // open the imu
      if (get_imu_data_start() < 0) {
        PCM_PRINT_ERROR("can not open the serial %s", uart1.c_str());
        return -1;
      }
      g_imu_is_ready = true;
      g_time_start = ros::Time::now();
      PCM_PRINT_INFO("start IMU!\n");
    }

    cv::cvtColor(frame, rgb, cv::COLOR_YUV2BGR_UYVY);

    // pub as ros msg
    std_msgs::Header header;
    header.frame_id = "camera_" + std::to_string(camera_id);
    header.stamp = ros::Time::now();

    // NOTE: compare the computer time and imu time
    if (g_imu_is_ready) {
      g_imu_t_mutex.lock();
      if (g_imu_is_ready) {
        ros::Duration diff = g_imu_time - header.stamp;
        std::cout << "diff is" << diff << std::endl;
      }
      g_imu_t_mutex.unlock();
    }

    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(header, "bgr8", rgb).toImageMsg();
    image_pub.publish(msg);
    PCM_PRINT_INFO(
        "image loop cost, %.2f ms(%.2f Hz), total cost: %.2f s, imu cnt: %u\n",
        timer.End(), 1000.0 / timer.End(), total.End() / 1000, g_imu_cnt);

    g_imu_cnt = 0;
  }

  cap.release();
  for (int i = 0; i < 5; ++i) {
    get_imu_data_stop();
    ros::Duration(0.0002).sleep();
  }

  Serial_Device_UnInit();
  PCM_PRINT_INFO("the node is shutdown!\n");
  return 0;
}
