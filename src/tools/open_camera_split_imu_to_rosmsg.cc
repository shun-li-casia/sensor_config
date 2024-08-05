/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: open_camera_spilt_imu_to_rosmsg.cc
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
#include "sensor_config/modules/imu_driver/serial_device.h"
#include "sensor_config/modules/imu_driver/imu_protocol.h"

#include "opencv2/imgproc.hpp"
#include "utility_tool/cmdline.h"
#include "utility_tool/print_ctrl_macro.h"
#include "utility_tool/pcm_debug_helper.h"
#include <cmath>

#include <ros/duration.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
#include <mutex>

struct imu_data {
  float gyr_x_{0.0f};
  float gyr_y_{0.0f};
  float gyr_z_{0.0f};
  float acc_x_{0.0f};
  float acc_y_{0.0f};
  float acc_z_{0.0f};
  uint16_t stamp_{0};
  uint32_t tp_{0u};
  uint32_t frame_id_{0u};
};

constexpr float g = 9.8015f;
constexpr float pi_div_180 = M_PI / 180.0f;

// t_imu = t_cam + time_shift
constexpr double time_shift = -0.036f;

static bool g_imu_is_ready = false;
static uint32_t g_imu_cnt = 0;
static unsigned int uart_baudrate = 1500000;

ros::Publisher g_imu_pub;
ros::Time g_time_start, g_imu_time;
std::mutex g_imu_t_mutex;

uint32_t g_last_tp = UINT32_MAX;
uint16_t g_last_stamp = UINT16_MAX;
bool g_is_first_frame = true;
constexpr double g_imu_t_step_s = 49.02 * 1e-6;

uint32_t g_imu_seq = 0, g_img_seq = 0;

ros::Time g_last_imu_time;
double ImuMsgCheckIntervalMs(const ros::Time& time_now) {
  if (g_last_imu_time.is_zero()) {
    g_last_imu_time = time_now;
    return 0.0f;
  }

  auto interval = time_now - g_last_imu_time;
  g_last_imu_time = time_now;
  return interval.toSec() * 1e3;
}

void ImuCallback(unsigned char* data_block, int data_block_len) {
  if (!g_imu_is_ready) {
    ros::Duration(0, 1e6);
    return;
  }

  struct imu_data data;

  data.gyr_x_ = (int16_t)(data_block[0] | (data_block[1] << 8)) * 1.0 * 0.025 *
                pi_div_180;
  data.gyr_y_ = (int16_t)(data_block[2] | (data_block[3] << 8)) * 1.0 * 0.025 *
                pi_div_180;
  data.gyr_z_ = (int16_t)(data_block[4] | (data_block[5] << 8)) * 1.0 * 0.025 *
                pi_div_180;
  data.acc_x_ =
      (int16_t)(data_block[6] | (data_block[7] << 8)) * 1.0 * 0.00025 * g;
  data.acc_y_ =
      (int16_t)(data_block[8] | (data_block[9] << 8)) * 1.0 * 0.00025 * g;
  data.acc_z_ =
      (int16_t)(data_block[10] | (data_block[11] << 8)) * 1.0 * 0.00025 * g;
  data.stamp_ = (uint16_t)(data_block[12] | (data_block[13] << 8));
  data.tp_ = (uint32_t)(data_block[14] | (data_block[15] << 8) |
                        (data_block[16] << 16) | (data_block[17] << 24));
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
    double tp_diff_ms = data.tp_ - g_last_tp;
    if (tp_diff_ms > 35) {
      PCM_PRINT_WARN("lost one imu frame!\n");
      time_diff_s = tp_diff_ms * 1e-3;
    } else {
      if (data.stamp_ > g_last_stamp) {
        uint16_t stamp_diff = data.stamp_ - g_last_stamp;
        time_diff_s = stamp_diff * g_imu_t_step_s;
      } else if (data.stamp_ < g_last_stamp) {
        uint16_t stamp_diff = 812 - g_last_stamp + 10 + data.stamp_;
        time_diff_s = stamp_diff * g_imu_t_step_s;
      }
    }

    g_imu_t_mutex.lock();
    g_imu_time += ros::Duration(time_diff_s);
    // std::cout << "data.tp_," << data.tp_ << ",data.stamp_," << data.stamp_
    //           << ","
    //           << "time_diff_s," << time_diff_s << ","
    //           << ImuMsgCheckIntervalMs(g_imu_time) << std::endl;
    ;
    g_imu_t_mutex.unlock();
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

  g_last_tp = data.tp_;
  g_last_stamp = data.stamp_;
  g_imu_cnt++;
}

int main(int argc, char* argv[]) {
  cmdline::parser par;

  par.add<int>("uav_id", 'u', "uav id", true, 0);
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

  int uav_id = par.get<int>("uav_id");
  int camera_id = par.get<int>("camera_id");
  ros::Publisher l_image_pub = nh.advertise<sensor_msgs::Image>(
      "cam_0", 1);
  ros::Publisher r_image_pub = nh.advertise<sensor_msgs::Image>(
      "cam_1", 1);
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
    cv::Mat frame, raw_img;
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
      g_imu_time = g_time_start;
      PCM_PRINT_INFO("start IMU!\n");
    }

    cv::cvtColor(frame, raw_img, cv::COLOR_YUV2GRAY_UYVY);

    // split
    int width = raw_img.cols;
    int height = raw_img.rows;
    cv::Rect leftRect(0, 0, width / 2, height);
    cv::Rect rightRect(width / 2, 0, width / 2, height);

    cv::Mat leftImage = raw_img(leftRect);
    cv::Mat rightImage = raw_img(rightRect);

    // pub as ros msg
    std_msgs::Header header;
    header.seq = g_img_seq++;

    // NOTE: compare the computer time and imu time
    g_imu_t_mutex.lock();
    if (g_imu_is_ready) {
      header.stamp = g_imu_time + ros::Duration(time_shift);
    } else {
      header.stamp = ros::Time::now();
    }
    g_imu_t_mutex.unlock();

    std_msgs::Header l_header = header;
    l_header.frame_id = "uav_" + std::to_string(uav_id) + "_cam_0";
    sensor_msgs::ImagePtr l_msg =
        cv_bridge::CvImage(l_header, "mono8", leftImage).toImageMsg();
    l_image_pub.publish(l_msg);

    std_msgs::Header r_header = header;
    r_header.frame_id = "uav_" + std::to_string(uav_id) + "_cam_1";
    sensor_msgs::ImagePtr r_msg =
        cv_bridge::CvImage(r_header, "mono8", rightImage).toImageMsg();
    r_image_pub.publish(r_msg);

    PCM_PRINT_INFO(
        "image loop cost, %.2f ms(%.2f Hz), total cost: %.2f s, imu cnt: %u\n",
        timer.End(), 1000.0 / timer.End(), total.End() / 1000, g_imu_cnt);

    g_imu_cnt = 0;
  }

  cap.release();

  Serial_Device_UnInit();
  PCM_PRINT_INFO("the node is shutdown!\n");
  return 0;
}
