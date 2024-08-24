#include <stdio.h>
#include <string.h>
#include "sensor_config/modules/imu_driver/imu_protocol.h"
#include "sensor_config/modules/imu_driver/serial_device.h"
#include "sensor_config/modules/imu_driver/serial_protocol.h"

int get_imu_data_start() {
  unsigned char pakage_data[BUFF_MAX_SIZE] = {0x5B, 0x01, 0x5D};
  int package_len = 3;

  // 发送数据
  return Serial_Send_Data(pakage_data, package_len);
}

int get_imu_data_stop() {
  unsigned char pakage_data[BUFF_MAX_SIZE] = {0x5B, 0x00, 0x5D};
  int package_len = 3;

  // 发送数据
  return Serial_Send_Data(pakage_data, package_len);
}

//0x02 red;
//0x03 green;
//0x04 blue;
//0x05 red green;
//0x06 red blue;
//0x07 green blue;
//0x08 red green blue;
//0x09 close;
int set_led_control(unsigned char num) {
  unsigned char pakage_data[BUFF_MAX_SIZE] = {0x5B, 0x00, 0x5D};
  int package_len = 3;
  pakage_data[1] = num;
  
  // 发送数据
  return Serial_Send_Data(pakage_data, package_len);
}

