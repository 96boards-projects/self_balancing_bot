/*
 * Title: IMU-MPU6050 interfacing with 96Boards CE
 * Author: Manivannan Sadhasivam
 * Copyright (c) 2017 Linaro Ltd.
 * All rights reserved.
 ********************************
 * 
 * I2C0 --> MPU6050
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <endian.h>
#include <math.h>
#include <sys/un.h>
#include "mraa/i2c.h"
#include "mraa/uart.h"
#include "pid.h"

#define I2C_BUS 0
#define UNIX_PATH_MAX 108

#ifndef FALSE
#define FALSE 0
#define TRUE (!FALSE)
#endif

#define KP 70
#define KI 1
#define KD 11

/* register definitions */
#define MPU6050_ADDR 0x68
#define MPU6050_REG_PWR_MGMT_1 0x6b
#define MPU6050_REG_SMPRT_DIV 0x19
#define MPU6050_REG_RAW_ACCEL_X 0x3b
#define MPU6050_REG_RAW_ACCEL_Y 0x3d
#define MPU6050_REG_RAW_ACCEL_Z 0x3f
#define MPU6050_REG_RAW_GYRO_X 0x43
#define MPU6050_REG_RAW_GYRO_Y 0x45
#define MPU6050_REG_RAW_GYRO_Z 0x47
#define MPU6050_REG_CONFIG 0x1a
#define MPU6050_REG_GYRO_CONFIG 0x1b
#define MPU6050_REG_ACCEL_CONFIG 0x1c
#define MPU6050_REG_USER_CTRL 0x6a

/* bit definitions */
#define MPU6050_WAKEUP 0x00
#define MPU6050_SIGNAL_RESET 0x01
#define MPU6050_SLEEP (1 << 6)
#define MPU6050_PLL_GYRO_X (1 << 0)
#define MPU6050_TEMP_DIS (1 << 3)
#define MPU6050_SRATE 0x00
#define MPU6050_FILTER 0x00
#define MPU6050_GYRO_FS_250 0x00
#define MPU6050_ACCEL_FS_2 0x00

/* accelerometer scale factor for (+/-)2g */
#define MPU6050_ACCEL_SCALE 16384.0

/* gyroscope scale factor for (+/-)250/s */
#define MPU6050_GYRO_SCALE 131.0

/* time delay (us) */
#define TIME_DELAY 20000

/* loop time (s) */
#define LTIME (TIME_DELAY * 0.000001)
 
mraa_uart_context uart;

volatile sig_atomic_t flag = TRUE;

void sig_handler(int signum)
{
	flag = FALSE;
}

float dist(int16_t a, int16_t b)
{
	  return sqrt((a*a) + (b*b));
}

float y_rotation(int16_t x, int16_t y, int16_t z)
{
	float radians;

	radians = atan2(x, dist(y, z));

	return -(radians * (180.0 / M_PI));
}

float x_rotation(int16_t x, int16_t y, int16_t z)
{
	float radians;

	radians = atan2(y, dist(x, z));
	
	return (radians * (180.0 / M_PI));
}

int16_t i2c_read_word(mraa_i2c_context dev, uint8_t command)
{
	return be16toh(mraa_i2c_read_word_data(dev, command));
}

int main(void) 
{
	mraa_result_t status = MRAA_SUCCESS;
	struct pid_ctrl pid;
    	struct sigaction action;
	uint8_t data;
	int ret;
	int16_t accel_data[3];
	int16_t gyro_data;
	int16_t accel_offset[3];
	int16_t gyro_offset;
	float x_rot;
	float angle_x = 0;
	float pid_val = 0;
	char buffer_x[10];
	char imu_data[20];
	
    	int baudrate = 9600, stopbits = 1, databits = 8;
        mraa_uart_parity_t parity = MRAA_UART_PARITY_NONE;
    	unsigned int ctsrts = FALSE, xonxoff = FALSE;
        const char *name = NULL, *dev_path = "/dev/ttyACM0";

    	memset(&action, 0, sizeof(struct sigaction));
    	action.sa_handler = sig_handler;
    	sigaction(SIGINT, &action, NULL);

	/* initialize PID controller */
	pid_init(KP, KI, KD, &pid);

	uart = mraa_uart_init_raw("/dev/ttyACM0");
	if (uart == NULL)
		goto err_exit;

	status = mraa_uart_settings(-1, &dev_path, &name, &baudrate, &databits, 
			&stopbits, &parity, &ctsrts, &xonxoff);
        if (status != MRAA_SUCCESS)
		goto err_exit;

	mraa_i2c_context i2c = mraa_i2c_init(I2C_BUS);
	if (i2c == NULL)
		goto err_exit;

    	status = mraa_i2c_address(i2c, MPU6050_ADDR);
    	if (status != MRAA_SUCCESS)
        	goto err_exit;

	/* reset the signal path */
	status = mraa_i2c_write_byte_data(i2c, MPU6050_SIGNAL_RESET, MPU6050_REG_USER_CTRL);
	if (status != MRAA_SUCCESS)
		goto err_exit;

	usleep(50000);

	/* configure power management regsiter */
	ret = mraa_i2c_read_byte_data(i2c, MPU6050_REG_PWR_MGMT_1);
	if (ret == -1)
		goto err_exit;
	
	data = ret;
	data |= (MPU6050_PLL_GYRO_X | MPU6050_TEMP_DIS);
        data &=	~(MPU6050_SLEEP);

	status = mraa_i2c_write_byte_data(i2c, data, MPU6050_REG_PWR_MGMT_1);
    	if (status != MRAA_SUCCESS)
        	goto err_exit;
	
	usleep(50000);

	/* 
	 * set 50Hz sample rate 
	 * sample rate = (gyro_rate / (1 + SMPRT_DIV)) = 1000 / (1+19) = 50Hz
	 */
	status = mraa_i2c_write_byte_data(i2c, MPU6050_SRATE, MPU6050_REG_SMPRT_DIV);
	if (status != MRAA_SUCCESS)
		goto err_exit;

        status = mraa_i2c_write_byte_data(i2c, MPU6050_FILTER, MPU6050_REG_CONFIG);
	if (status != MRAA_SUCCESS)
		goto err_exit;

	/* set 250/s full scale range */
        status = mraa_i2c_write_byte_data(i2c, MPU6050_GYRO_FS_250, MPU6050_REG_GYRO_CONFIG);
	if (status != MRAA_SUCCESS)
		goto err_exit;

	/* set 2g full scale range */
        status = mraa_i2c_write_byte_data(i2c, MPU6050_ACCEL_FS_2, MPU6050_REG_ACCEL_CONFIG);
	if (status != MRAA_SUCCESS)
		goto err_exit;

	usleep(2000000);

	/* read accel data for calculating offset */
	accel_offset[0] = i2c_read_word(i2c, MPU6050_REG_RAW_ACCEL_X) / MPU6050_ACCEL_SCALE;
	accel_offset[1] = i2c_read_word(i2c, MPU6050_REG_RAW_ACCEL_Y) / MPU6050_ACCEL_SCALE;
	accel_offset[2] = i2c_read_word(i2c, MPU6050_REG_RAW_ACCEL_Z) / MPU6050_ACCEL_SCALE;

        /* read gyro data for calculating offset */
        gyro_offset = i2c_read_word(i2c, MPU6050_REG_RAW_GYRO_X) / MPU6050_GYRO_SCALE;

	while(flag) {
		/* read raw accel data */
		accel_data[0] = i2c_read_word(i2c, MPU6050_REG_RAW_ACCEL_X) / MPU6050_ACCEL_SCALE;
		accel_data[1] = i2c_read_word(i2c, MPU6050_REG_RAW_ACCEL_Y) / MPU6050_ACCEL_SCALE;
		accel_data[2] = i2c_read_word(i2c, MPU6050_REG_RAW_ACCEL_Z) / MPU6050_ACCEL_SCALE;
	
		/* read raw gyro data */
		gyro_data = i2c_read_word(i2c, MPU6050_REG_RAW_GYRO_X) / MPU6050_GYRO_SCALE;

		accel_data[0] -= accel_offset[0];
		accel_data[1] -= accel_offset[1];
		accel_data[2] -= accel_offset[2];

		gyro_data -= gyro_offset;

		/* calculate x axis rotation */
	        x_rot = x_rotation(accel_data[0], accel_data[1], accel_data[2]);

		/* implement complementary filter for sensor fusion */
		angle_x = (0.98)*(angle_x + (gyro_data * LTIME)) + (0.02 * x_rot);

		/* calculate PID step value */
		pid_val = pid_step(angle_x, 0, LTIME, &pid);
		if (pid_val > 255)
			pid_val = 255;
		else if (pid_val < -255)
			pid_val = -255;

		memset(buffer_x, 0, sizeof(buffer_x));
		memset(imu_data, 0, sizeof(imu_data));

		if (pid_val < 0.0) {
			sprintf(buffer_x, "%f", -pid_val);
			strcat(imu_data, "r:");
			strcat(imu_data, buffer_x);
			strcpy(&imu_data[19], "\n");
		} else if (pid_val > 0.0) {
                        sprintf(buffer_x, "%f", pid_val);
			strcat(imu_data, "f:");
			strcat(imu_data, buffer_x);
			strcpy(&imu_data[19], "\n");
		}
		
		mraa_uart_write(uart, (const char *)imu_data, sizeof(imu_data));

		usleep(TIME_DELAY);
	}

	mraa_uart_stop(uart);

	return 0;

err_exit:
	return -1;
}
