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

static bool g_imu_is_ready = false;
static uint32_t g_imu_cnt = 0;
static char uart1[] = "/dev/ttyUSB0";
static unsigned int uart_baudrate = 1500000;

ros::Publisher g_imu_pub;
ros::Time g_time_start;

struct imu_data {
  uint16_t DIAG_STAT;
  uint16_t STAMP;
  uint32_t TIMESTAMP;
  uint32_t TIMESTAMP_25HZ;
  float X_GYRO_OUT;
  float Y_GYRO_OUT;
  float Z_GYRO_OUT;
  float X_ACCL_OUT;
  float Y_ACCL_OUT;
  float Z_ACCL_OUT;
  float TEMP_OUT;
};

void ImuCallback(unsigned char* data_block, int data_block_len) {
  if (!g_imu_is_ready) {
    ros::Duration(0, 1e6);
    return;
  }

  struct imu_data data;
  data.DIAG_STAT = (int16_t)(data_block[0] | (data_block[1] << 8));
  data.X_GYRO_OUT =
      (int16_t)(data_block[2] | (data_block[3] << 8)) * 1.0 * 0.025;
  data.Y_GYRO_OUT =
      (int16_t)(data_block[4] | (data_block[5] << 8)) * 1.0 * 0.025;
  data.Z_GYRO_OUT =
      (int16_t)(data_block[6] | (data_block[7] << 8)) * 1.0 * 0.025;
  data.X_ACCL_OUT =
      (int16_t)(data_block[8] | (data_block[9] << 8)) * 1.0 * 0.00025;
  data.Y_ACCL_OUT =
      (int16_t)(data_block[10] | (data_block[11] << 8)) * 1.0 * 0.00025;
  data.Z_ACCL_OUT =
      (int16_t)(data_block[12] | (data_block[13] << 8)) * 1.0 * 0.00025;
  data.TEMP_OUT = (int16_t)(data_block[14] | (data_block[15] << 8)) * 1.0 * 0.1;
  data.STAMP = (int16_t)(data_block[16] | (data_block[17] << 8));
  data.TIMESTAMP = (uint32_t)(data_block[18] | (data_block[19] << 8) |
                              (data_block[20] << 16) | (data_block[21] << 24));
  data.TIMESTAMP_25HZ =
      (uint32_t)(data_block[22] | (data_block[23] << 8) |
                 (data_block[24] << 16) | (data_block[25] << 24));

  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = "body";
  ros::Duration offset(0, data.TIMESTAMP * 1e6);
  imu_msg.header.stamp = g_time_start + offset;
  imu_msg.header.seq = data.STAMP;

  imu_msg.angular_velocity.x = data.X_GYRO_OUT;
  imu_msg.angular_velocity.y = data.Y_GYRO_OUT;
  imu_msg.angular_velocity.z = data.Z_GYRO_OUT;
  imu_msg.linear_acceleration.x = data.X_ACCL_OUT;
  imu_msg.linear_acceleration.y = data.Y_ACCL_OUT;
  imu_msg.linear_acceleration.z = data.Z_ACCL_OUT;

  g_imu_pub.publish(imu_msg);
  g_imu_cnt++;
}

int main(int argc, char* argv[]) {
  cmdline::parser par;

  par.add<int>("camera_id", 'c', "camera id", false, 0);
  par.parse_check(argc, argv);

  ros::init(argc, argv, "open_camera_imu_to_rosmsg_node");
  ros::NodeHandle nh;

  // init imu
  Set_Serial_Parse_Callback(ImuCallback);
  Serial_Device_Init(uart1, uart_baudrate);

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
        PCM_PRINT_ERROR("can not open the serial %s", uart1);
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
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(header, "bgr8", rgb).toImageMsg();
    image_pub.publish(msg);
    PCM_PRINT_INFO(
        "image loop cost, %.2f ms(%.2f Hz), total cost: %.2f s, imu cnt: %u\n",
        timer.End(), 1000.0 / timer.End(), total.End() / 1000, g_imu_cnt);

    g_imu_cnt = 0;
  }

  cap.release();
  get_imu_data_stop();
  Serial_Device_UnInit();  // 串口设备注销
  PCM_PRINT_INFO("the node is shutdown!\n");
  return 0;
}
