#include "mpu6050.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

int mpu6050_init(){

  HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, (DEVICE_ADDRESS<<1)+0, 1, 100);
  if(ret==HAL_OK){
	  printf("Device connected\n"); //device ready
  }else{
	  printf("Unable to connect to device\n"); //device not ready
	  return ret;

  }

  uint8_t temp_data = FS_GYRO_500;
  ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS<<1)+0, REG_CONFIG_GYRO, 1, &temp_data, 1, 100);
  if(ret==HAL_OK){
	  printf("Configuring Gyro\n");//write_success
	}else{
		printf("Unable to Configure Gyro\n");//write_fail
  		return ret;
	}

  temp_data = FS_ACC_4G;
  ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS<<1)+0, REG_CONFIG_ACC, 1, &temp_data, 1, 100);
    if(ret ==HAL_OK){
    	printf("Configuring Accelerometer\n");//write_success
  	}else{
  		printf("Unable to configure Accelerometer\n");//write_fail
  		return ret;
  	}

    temp_data = 0;
    ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS<<1)+0, REG_USR_CTRL, 1, &temp_data, 1, 100);
	if(ret==HAL_OK){
		printf("Exiting from sleep mode\n");//write_success
	}else{
		printf("Unable to exit sleep mode\n");//write_fail
		return ret;
	}

	temp_data = DATA_READY_INTERRUPT; //Data ready interrupt
	ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS<<1)+0, INTERRUPT_REGISTER, 1, &temp_data, 1, 100);
	if(ret==HAL_OK){
		printf("Enabling Interrupt\n");//write_success
	}else{
		printf("Failed to enable interrupt\n");//write_fail
	}
	return ret;

}

int16_t mpu6050_read(int16_t i){
	uint8_t data[2];
	int16_t x_acc;
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS<<1)+1, REG_DATA, 1, data, 2, 100);
	x_acc = (int16_t)data[0] << 8 + data[1];
	return x_acc;
	if(i<100){
		i++;
		return i;
	}
	else
	{
		return 0;
	}
//	printf("X_acc: %d\n",x_acc);
}
