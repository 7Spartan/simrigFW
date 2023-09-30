#ifndef __COMMON_DATA_H
#define __COMMON_DATA_H


enum motor_dir{
	cw,
	ccw
};

typedef enum{
	imuinitpass = 0x04U,
	imuinitfail = 0x05U,
	gyroinitpass = 0x06U,
	gyroinitfail = 0x07U,
	accconfigpass = 0x08U,
	accconfigfail = 0x09U,
	exitsleepmodeok = 0x0AU,
	exitsleepmodefail = 0x0BU,
	interruptok = 0x0CU,
	interruptfail = 0x0DU,
}imuStatus;

typedef struct dsacc{
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
}dsacc;

typedef struct dsgyro{
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
}dsgyro;

#endif
