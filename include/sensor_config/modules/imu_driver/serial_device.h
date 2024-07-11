#ifndef __SERIAL_DEVICE_H__
#define __SERIAL_DEVICE_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#define SERIAL_DATA_MAX_LEN 1024 * 4  // 4K

typedef void (*CALLBACK_PARSE)(unsigned char* data_block, int data_block_len);

int Serial_Device_Init(const char* uart, unsigned int baudrate);  // 串口设备初始化

int Set_Serial_Parse_Callback(
    CALLBACK_PARSE callback_ptr);  // 设置串口设备数据解析回调

int Serial_Send_Data(unsigned char* data, int data_len);  // 发送数据到串口

int Serial_Device_UnInit();  // 关闭串口设备

#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */
