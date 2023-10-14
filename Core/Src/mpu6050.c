#include "mpu6050.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

imuStatus mpu6050_init() {

	imuStatus ret = HAL_I2C_IsDeviceReady(&hi2c1,(DEVICE_ADDRESS << 1) + 0, 1, 100);
	if (ret == HAL_OK) {
//		printf("Device connected\n"); //device ready
	} else {
//		printf("Unable to connect to device\n"); //device not ready
		return imuinitfail;
	}

	uint8_t temp_data = FS_GYRO_500;
	ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1) + 0, REG_CONFIG_GYRO,1, &temp_data, 1, 100);
	if (ret == HAL_OK) {
//		printf("Configuring Gyro\n"); //write_success
	} else {
//		printf("Unable to Configure Gyro\n"); //write_fail
		return gyroinitfail;
	}

	temp_data = FS_ACC_4G;
	ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1) + 0, REG_CONFIG_ACC,1, &temp_data, 1, 100);
	if (ret == HAL_OK) {
//		printf("Configuring Accelerometer\n"); //write_success
	} else {
//		printf("Unable to configure Accelerometer\n"); //write_fail
		return accconfigfail;
	}

	temp_data = 0;
	ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1) + 0, REG_USR_CTRL, 1,&temp_data, 1, 100);
	if (ret == HAL_OK) {
//		printf("Exiting from sleep mode\n"); //write_success
	} else {
//		printf("Unable to exit sleep mode\n"); //write_fail
		return exitsleepmodefail;
	}

	temp_data = DATA_READY_INTERRUPT; //Data ready interrupt
	ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1) + 0, INTERRUPT_REGISTER, 1, &temp_data, 1, 100);
	if (ret == HAL_OK) {
//		printf("Enabling Interrupt\n"); //write_success
	} else {
//		printf("Failed to enable interrupt\n"); //write_fail
		return interruptfail;
	}

	return HAL_OK;

}

void mpu6050_imu_read() {
	if(!i2cReadInProgress){
		i2cReadInProgress=1;
		i2c_status = HAL_I2C_Mem_Read_DMA(&hi2c1, (DEVICE_ADDRESS << 1) + 1, ACC_REG_DATA, 1, imu_data, 14);
	}
}

