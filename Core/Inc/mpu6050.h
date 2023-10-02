#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stdint.h"
#include "common_data.h"

extern uint8_t accel_data[6];
extern uint8_t gyro_data[6];
extern int i2c_status;
extern volatile uint8_t i2cReadInProgress;

#define DEVICE_ADDRESS 0X68

#define FS_GYRO_250	0
#define FS_GYRO_500	8
#define FS_GYRO_1000	16
#define FS_GYRO_2000	24

#define FS_ACC_2G	0
#define FS_ACC_4G	8
#define FS_ACC_8G	16
#define FS_ACC_16G	24

#define REG_CONFIG_GYRO	27
#define REG_CONFIG_ACC	28
#define REG_USR_CTRL	107 //Temperature sensor
#define INTERRUPT_REGISTER	56
#define ACC_REG_DATA	59
#define GYRO_REG_DATA	67
#define DATA_READY_INTERRUPT	1

imuStatus mpu6050_init();
void mpu6050_accel_read();
void mpu6050_gyro_read();
#endif
