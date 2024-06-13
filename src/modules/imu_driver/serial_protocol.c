#include <stdio.h>
#include <string.h>
#include "sensor_config/imu_driver/serial_protocol.h"

static unsigned short checksum(unsigned char* data, int len) {
  int i = 0;
  unsigned short crc = 0x00;  // 从数据头开始校验
  for (i = 0; i < len; i++) {
    crc += data[i];
  }
  return crc;
}

static void DataTransfer(unsigned char* data, int len, unsigned char* output,
                         int* output_len) {
  int i = 0, j = 0;
  output[j++] = 0x5B;  // 报文头
  for (i = 0; i < len; i++) {
    if (data[i] == 0x5B) {
      output[j++] = 0x5C;
      output[j++] = 0x01;
    } else if (data[i] == 0x5C) {
      output[j++] = 0x5C;
      output[j++] = 0x00;
    } else if (data[i] == 0x5D) {
      output[j++] = 0x5C;
      output[j++] = 0x02;
    } else {
      output[j++] = data[i];
    }
  }
  output[j++] = 0x5D;  // 报文尾
  *output_len = j;
}

static int DataUnTransfer(unsigned char* data, int len, unsigned char* output,
                          int* output_len) {
  int i = 0, j = 0;
  for (i = 1; i < len - 1; i++)  // 不处理报文头和报文尾
  {
    if (data[i] == 0x5C) {
      if (data[i + 1] == 0x00) {
        output[j++] = 0x5C;
        i++;
      } else if (data[i + 1] == 0x01) {
        output[j++] = 0x5B;
        i++;
      } else if (data[i + 1] == 0x02) {
        output[j++] = 0x5D;
        i++;
      } else  // 0x5C 后必须为 00 01 02 否则为非法值
      {
        printf("Function:%s Line:%d ERROR Data is illegel!\n", __FUNCTION__,
               __LINE__);
        return -1;
      }
    } else {
      output[j++] = data[i];
    }
  }
  *output_len = j;
  return 0;
}

int DataUnPackage(unsigned char* package_data, int package_size,
                  CALLBACK_PARSE data_parse_call_back) {
  if (NULL == package_data) {
    printf("Function:%s Line:%d DataUnPackage ERROR: package_data is NULL!\n",
           __FUNCTION__, __LINE__);
    return -1;
  }

  unsigned char buffer[BUFF_MAX_SIZE] = {0};  // 数据缓存区
  int buff_data_len = 0;                      // 数据缓存区数据长度
  unsigned short crc = 0;                     // 校验位
  int data_len = 0;                           // 数据长度
  int ret = -1;

  if (package_data[0] != 0x5B ||
      package_data[package_size - 1] != 0x5D)  // 判断头和尾
  {
    printf(
        "Function:%s Line:%d DataUnPackage ERROR: The head or tail is ERROR!\n",
        __FUNCTION__, __LINE__);
    return -1;
  }

  ret = DataUnTransfer(package_data, package_size, buffer,
                       &buff_data_len);  // 数据反转义去除头、尾
  if (ret == -1) {
    printf("Function:%s Line:%d DataUnPackage ERROR: DataUnTransfer fail!\n",
           __FUNCTION__, __LINE__);
    return -1;
  }

  // for(int k = 0; k < buff_data_len; k++)
  //{
  //     printf("0x%x ", buffer[k]);
  //}
  // printf(" buffer_len: %d\n", buff_data_len);
  crc = checksum(buffer, buff_data_len - 2);  // 计算校验位
  // printf("crc: %x %x %x\r\n",crc, buffer[buff_data_len - 1],
  // buffer[buff_data_len -2]);

  if (crc != (buffer[buff_data_len - 2] |
              (buffer[buff_data_len - 1] << 8)))  // 判断校验位
  {
    printf("Function:%s Line:%d DataUnPackage ERROR: Xor is ERROR!\n",
           __FUNCTION__, __LINE__);
    return -1;
  }

  data_parse_call_back(buffer, buff_data_len - 2);  // 回调数据块解析

  return 0;
}
