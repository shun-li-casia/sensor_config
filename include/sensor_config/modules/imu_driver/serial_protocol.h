#ifndef __SERIAL_PROTOCOL_H__
#define __SERIAL_PROTOCOL_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "sensor_config/modules/imu_driver/serial_device.h"

#define BUFF_MAX_SIZE 1024  // 1K

/****************************************   函数声明
 * ****************************************************/
// 公共函数

int DataUnPackage(unsigned char* package_data, int package_size,
                  CALLBACK_PARSE data_parse_call_back);  // 数据解包

#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */
