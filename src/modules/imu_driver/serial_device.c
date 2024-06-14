#include <stdio.h>  /*标准输入输出定义*/
#include <stdlib.h> /*标准函数库定义*/
#include <unistd.h> /*Unix 标准函数定义*/
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <errno.h>   /*错误号定义*/
#include <string.h>
#include <pthread.h>
#include <signal.h>

#include "sensor_config/imu_driver/serial_device.h"
#include "sensor_config/imu_driver/serial_protocol.h"

static int fd_uart = -1;

static pthread_mutex_t uart_mutex_lock;  // 互斥锁

static pthread_t pthread_uart_rcv;

static int uart_running_flag = 0;

CALLBACK_PARSE uart_callback = NULL;

static void* UartRecv_Proc(void* arg) {
  int ret = 0;
  fd_set fdset;
  struct timeval timeout = {3, 0};  // 设置超时时长
  unsigned char buffer[SERIAL_DATA_MAX_LEN] = {
      0};                        // 用于存放符合协议的串口数据
  int data_index = 0;            // 串口数据下标
  unsigned char start_flag = 0;  // 开始记录数据的标志

  printf("Function:%s Line:%d Creat pthread_uart1_rcv success!.......\n",
         __FUNCTION__, __LINE__);

  uart_running_flag = 1;
  while (uart_running_flag) {
    FD_ZERO(&fdset);
    FD_SET(fd_uart, &fdset);
    timeout.tv_sec = 3;  // 重新赋值
    ret = select(fd_uart + 1, &fdset, NULL, NULL, &timeout);
    if (ret <= 0) {
      continue;  // 未接收到数据 继续监听
    } else {
      if (FD_ISSET(fd_uart, &fdset))  // 是否收到数据
      {
        unsigned char ch = 0;
        ret = 1;
        while (ret > 0) {
          ret = read(fd_uart, &ch, 1);
          if (ret <= 0) {
            break;
          }

          if (ch == 0x5B)  // 遇到数据头
          {
            start_flag = 1;                     // 开始记录数据
            memset(buffer, 0, sizeof(buffer));  // 清空记录的数据
            data_index = 0;                     // 下标长度清空
          } else if (ch == 0x5D)                // 遇到数据尾
          {
            if (start_flag)  // 且已经记录数据
            {
              buffer[data_index++] = ch;
              DataUnPackage(buffer, data_index, uart_callback);  // 数据解包
              start_flag = 0;  // 停止记录数据
            }
          }

          if (start_flag)  // 如果开始记录数据
          {
            if (data_index >= SERIAL_DATA_MAX_LEN - 1)  // 如果数据长度越界
            {
              printf(
                  "Error: Serial1 Data is too long, and cannot exceed %d "
                  "bytes.\n",
                  SERIAL_DATA_MAX_LEN);
              start_flag = 0;  // 停止记录数据
            } else
              buffer[data_index++] = ch;
          }
        }
      }
    }
  }
  return NULL;
}

static int Uart_Set(int fd, int speed, int flow_ctrl, int databits,
                    int stopbits, int parity) {
  int i;
  int status;
  int speed_arr[] = {B2000000, B1500000, B1152000, B1000000, B921600, B576000,
                     B500000,  B460800,  B230400,  B115200,  B19200,  B9600,
                     B4800,    B2400,    B1200,    B300};

  int name_arr[] = {2000000, 1500000, 1152000, 1000000, 921600, 576000,
                    500000,  460800,  230400,  115200,  19200,  9600,
                    4800,    2400,    1200,    300};

  struct termios options;

  /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
   */
  if (tcgetattr(fd, &options) != 0) {
    perror("SetupSerial 1");
    return (-1);
  }
  int seted_flag = 0;
  // 设置串口输入波特率和输出波特率
  for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
    printf("------ baudrate Speed = %d\n", name_arr[i]);
    if (speed == name_arr[i]) {
      cfsetispeed(&options, speed_arr[i]);
      cfsetospeed(&options, speed_arr[i]);
      seted_flag = 1;
      printf("Setting baudrate!!\n");
      break;
    }
  }

  if (seted_flag == 0) {
    printf("Unknow baudrate!!\n");
    return -1;
  }
  // 修改控制模式，保证程序不会占用串口
  options.c_cflag |= CLOCAL;
  // 修改控制模式，使得能够从串口中读取输入数据
  options.c_cflag |= CREAD;

  // 设置数据流控制
  switch (flow_ctrl) {
    case 0:  // 不使用流控制
      options.c_cflag &= ~CRTSCTS;
      break;

    case 1:  // 使用硬件流控制
      options.c_cflag |= CRTSCTS;
      break;
    case 2:  // 使用软件流控制
      options.c_cflag |= IXON | IXOFF | IXANY;
      break;
  }
  // 设置数据位

  // 屏蔽其他标志位
  options.c_cflag &= ~CSIZE;
  switch (databits) {
    case 5:
      options.c_cflag |= CS5;
      break;
    case 6:
      options.c_cflag |= CS6;
      break;
    case 7:
      options.c_cflag |= CS7;
      break;
    case 8:
      options.c_cflag |= CS8;
      break;
    default:
      fprintf(stderr, "Unsupported data size\n");
      return (-1);
  }
  // 设置校验位
  switch (parity) {
    case 'n':
    case 'N':  // 无奇偶校验位。
      options.c_cflag &= ~PARENB;
      options.c_iflag &= ~INPCK;
      break;
    case 'o':
    case 'O':  // 设置为奇校验
      options.c_cflag |= (PARODD | PARENB);
      options.c_iflag |= INPCK;
      break;
    case 'e':
    case 'E':  // 设置为偶校验
      options.c_cflag |= PARENB;
      options.c_cflag &= ~PARODD;
      options.c_iflag |= INPCK;
      break;
    case 's':
    case 'S':  // 设置为空格
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      break;
    default:
      fprintf(stderr, "Unsupported parity\n");
      return (-1);
  }
  // 设置停止位
  switch (stopbits) {
    case 1:
      options.c_cflag &= ~CSTOPB;
      break;
    case 2:
      options.c_cflag |= CSTOPB;
      break;
    default:
      fprintf(stderr, "Unsupported stop bits\n");
      return (-1);
  }

  // 修改输出模式，原始数据输出
  options.c_oflag &= ~OPOST;

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_lflag &= ~(ISIG | ICANON);
  options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

  // cfmakeraw(&options);
  // 设置等待时间和最小接收字符
  options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
  options.c_cc[VMIN] = 1;  /* 读取字符的最少个数为1 */

  // 如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
  tcflush(fd, TCIFLUSH);

  // 激活配置 (将修改后的termios数据设置到串口中）
  if (tcsetattr(fd, TCSANOW, &options) != 0) {
    perror("com set error!\n");
    return -1;
  }
  return 0;
}

static int Uart_Init(const char* uart, unsigned int baudrate)  // 串口初始化
{
  fd_uart = open(uart, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_uart > 0) {
    int ret = Uart_Set(fd_uart, baudrate, 0, 8, 1, 'N');
    if (ret == 0) {
      printf("----------UART1(%s) Baudrate = %d\n", uart, baudrate);
      if (pthread_mutex_init(&uart_mutex_lock, NULL) == -1) {
        printf("Function:%s Line:%d  ERROR: Init uart1_mutex_lock fail!\n",
               __FUNCTION__, __LINE__);
        return -1;
      }
      printf("Function:%s Line:%d Init uart_mutex_lock success!\n",
             __FUNCTION__, __LINE__);

      if (pthread_create(&pthread_uart_rcv, NULL, UartRecv_Proc, NULL) !=
          0)  // 开启数据接收线程
      {
        printf("Function:%s Line:%d  ERROR: Creat pthread_uart_rcv fail!\n",
               __FUNCTION__, __LINE__);
        return -1;
      }
      return 0;
    }
    printf("Function:%s Line:%d  ERROR: uart Set fail!\n", __FUNCTION__,
           __LINE__);
  }
  printf("Function:%s Line:%d  ERROR: uart open fail!\n", __FUNCTION__,
         __LINE__);
  return -1;
}

int Serial_Device_Init(const char* uart, unsigned int baudrate)  // 串口设备初始化
{
  if (uart_callback != NULL) {
    return Uart_Init(uart, baudrate);  // MCU 串口
  }

  return -1;
}

int Set_Serial_Parse_Callback(
    CALLBACK_PARSE callback_ptr)  // 设置串口设备数据解析回调
{
  if (callback_ptr == NULL) return -1;
  uart_callback = callback_ptr;
  return 0;
}

int Serial_Device_UnInit() {
  if (uart_running_flag != 0)  // 退出线程
  {
    uart_running_flag = 0;
    if (pthread_uart_rcv != 0) {
      pthread_join(pthread_uart_rcv, NULL);
    }
  }
  if (fd_uart > 0)  // 关闭串口
    close(fd_uart);
}

int Serial_Send_Data(unsigned char* data, int data_len) {
  if (data_len <= 0 || data == NULL) {
    return -1;
  }

  if (fd_uart > 0) {
    pthread_mutex_lock(&uart_mutex_lock);  // 加锁
    write(fd_uart, (const void*)data, data_len);
    pthread_mutex_unlock(&uart_mutex_lock);  // 解锁
  } else {
    printf(
        "---------------Function:%s Line:%d  ERROR: dev_type error or serial "
        "not be open!\n",
        __FUNCTION__, __LINE__);
    return -1;
  }
  return 0;
}
