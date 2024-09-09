/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: open_camera_spilt_imu_to_rosmsg_v4l2.cc
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
#include "utility_tool/file_writter.h"
#include "utility_tool/system_lib.h"
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
#include <atomic>

#include <V4l2Capture.h>
#include <V4l2Device.h>

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
constexpr double time_shift = -0.036f;
// NOTE: 48.6710
constexpr double g_imu_t_step_s = 48.79 * 1e-6;

std::atomic<bool> g_imu_is_ready;
static unsigned int uart_baudrate = 1500000;

ros::Publisher g_imu_pub;
ros::Time g_time_start, g_imu_time;
std::mutex g_imu_t_mutex;

uint16_t g_last_stamp = UINT16_MAX, g_max_stamp = 0;
bool g_is_first_frame = true;

uint32_t g_imu_seq = 0, g_img_seq = 0;
char* g_imu_env_value = NULL;
utility_tool::FileWritter::Ptr g_imu_writter, g_img_writter;

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
        time_diff_s = stamp_diff * g_imu_t_step_s;
      } else if (data.stamp_ < g_last_stamp) {
        uint16_t stamp_diff = g_max_stamp - g_last_stamp + 10 + data.stamp_;
        time_diff_s = stamp_diff * g_imu_t_step_s;
      }

      // update the last stamp
      g_last_stamp = data.stamp_;
    } else {
      g_imu_t_mutex.lock();
      g_imu_writter->Write(g_imu_time.toSec(), data.stamp_, time_diff_s,
                           data.acc_x_, data.acc_y_, data.acc_z_, data.gyr_x_,
                           data.gyr_y_, data.gyr_z_);
      g_imu_t_mutex.unlock();
      return;
    }

    g_imu_t_mutex.lock();
    g_imu_time += ros::Duration(time_diff_s);
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
}

int main(int argc, char* argv[]) {
  cmdline::parser par;

  par.add<int>("uav_id", 'u', "uav id", true, 0);
  par.add<int>("imu_uart", 'i', "imu_uart id", true, 0);
  par.add<int>("camera_id", 'c', "camera id", true, 0);
  par.add<int>("if_gray", 'g', "use gray image", true, 0);
  par.parse_check(argc, argv);

  bool if_gray = par.get<int>("if_gray");

  ros::init(argc, argv, "open_camera_imu_to_rosmsg_node");
  ros::NodeHandle nh;

  // init imu
  g_imu_is_ready.store(false);
  g_imu_env_value = getenv("UAV_ID");
  g_imu_writter = std::make_shared<utility_tool::FileWritter>(
      "imu_debug_" + utility_tool::GetCurLocalTimeStr("%Y%m%d%H%M%S") + ".csv",
      6);
  g_imu_writter->SetDelimiter(",");
  g_imu_writter->EraseOpen();

  std::string uart1 = "/dev/ttyUSBIMU";
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

  if (*g_imu_env_value >= '0' && *g_imu_env_value <= '7') {
    set_led_control(*g_imu_env_value - '0' + 2);
  } else {
    PCM_PRINT_ERROR("please check the UAV_ID NUM is between 0 to 7!!!\n");
  }

  int uav_id = par.get<int>("uav_id");
  int camera_id = par.get<int>("camera_id");
  ros::Publisher l_image_pub = nh.advertise<sensor_msgs::Image>("cam_0", 1);
  ros::Publisher r_image_pub = nh.advertise<sensor_msgs::Image>("cam_1", 1);
  g_imu_pub = nh.advertise<sensor_msgs::Imu>("imu_raw_0", 10000);

  // init the camera
  const std::string in_devname = "/dev/video" + std::to_string(camera_id);
  V4L2DeviceParameters mparam(in_devname.c_str(), V4L2_PIX_FMT_UYVY, 3840, 1080,
                              25, 0, 0);
  V4l2Capture* videoCapture;

  for (int i = 0; i < 10; ++i) {
    videoCapture = V4l2Capture::create(mparam, V4l2Access::IOTYPE_MMAP);
    if (!videoCapture) {
      PCM_PRINT_ERROR("create videoCapture failed!\n");
      ros::Duration(1).sleep();
      continue;
    } else {
      PCM_PRINT_INFO("create videoCapture successfully!\n");
      break;
    }
  }

  if (!videoCapture) {
    PCM_PRINT_ERROR("create videoCapture failed!\n");
    return -1;
  }

  timeval capture_time_out = {0, 400000};  // 40 millisecond

  g_img_writter = std::make_shared<utility_tool::FileWritter>(
      "img_debug_" + utility_tool::GetCurLocalTimeStr("%Y%m%d%H%M%S") + ".csv",
      6);
  g_img_writter->SetDelimiter(",");
  g_img_writter->EraseOpen();

  const int camera_is_stable = 10;
  uint32_t count = 0;

  ros::Time last_img_time(0);
  utility_tool::Timer t_cap, t_res;
  ros::Rate r(400);
  while (ros::ok()) {
    cv::Mat frame, raw_img;
    t_cap.Start();
    int ret = videoCapture->isReadable(&capture_time_out);
    if (ret == -1) {
      PCM_PRINT_ERROR("read frame error!\n");
      continue;
    } else if (ret == 0) {
      continue;
    } else {
      ret = videoCapture->read(frame);
      if (ret != 0) {
        continue;
      }
    }
    float cap_time = t_cap.End() / 1000.0f;

    t_res.Start();

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
      g_imu_is_ready.store(true);
      g_time_start = ros::Time::now();
      g_imu_time = g_time_start;
      PCM_PRINT_INFO("start IMU!\n");
    }

    if (if_gray) {
      cv::cvtColor(frame, raw_img, cv::COLOR_YUV2GRAY_UYVY);
    } else {
      cv::cvtColor(frame, raw_img, cv::COLOR_YUV2BGR_UYVY);
    }

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

    float res_time = t_res.End() / 1000.0f;
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
    sensor_msgs::ImagePtr l_msg;
    if (if_gray) {
      l_msg = cv_bridge::CvImage(l_header, "mono8", leftImage).toImageMsg();
    } else {
      l_msg = cv_bridge::CvImage(l_header, "bgr8", leftImage).toImageMsg();
    }
    l_image_pub.publish(l_msg);

    std_msgs::Header r_header = header;
    r_header.frame_id = "uav_" + std::to_string(uav_id) + "_cam_1";
    sensor_msgs::ImagePtr r_msg;
    if (if_gray) {
      r_msg = cv_bridge::CvImage(r_header, "mono8", rightImage).toImageMsg();
    } else {
      r_msg = cv_bridge::CvImage(r_header, "bgr8", rightImage).toImageMsg();
    }
    r_image_pub.publish(r_msg);

    ros::Time time_now = ros::Time::now();
    PCM_STREAM_DEBUG("image header tp: "
                         << l_msg->header.stamp
                         << " ros current time: " << time_now << " diff time: "
                         << l_msg->header.stamp - time_now << std::endl;);

    PCM_PRINT_INFO("img tp: %lf, diff: %lf\n", l_msg->header.stamp.toSec(),
                   (l_msg->header.stamp - last_img_time).toSec());

    g_img_writter->Write(ros::Time::now().toSec(),
                         (l_msg->header.stamp - last_img_time).toSec(),
                         cap_time, res_time / 1000.0f);
    last_img_time = l_msg->header.stamp;

    r.sleep();
  }

  if (videoCapture->stop()) {
    PCM_PRINT_INFO("stop video capture!\n");
  } else {
    PCM_PRINT_WARN("stop video failed!\n");
  }
  delete videoCapture;

  Serial_Device_UnInit();
  PCM_PRINT_INFO("the node is shutdown!\n");
  return 0;
}
