#ifndef __IMU_PROTOCOL_H__
#define __IMU_PROTOCOL_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */


int get_imu_data_start();
int get_imu_data_stop();

int set_led_control(unsigned char num);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __IMU_PROTOCOL_H__ */
