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
#include <time.h>
#include "mraa/i2c.h"
#include "mraa/uart.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define I2C_BUS 0
#define UNIX_PATH_MAX 108

#ifndef FALSE
#define FALSE 0
#define TRUE (!FALSE)
#endif

#define TIME_DELAY 2000
#define LTIME (TIME_DELAY * 0.000001)
 
mraa_uart_context uart;
MPU6050 mpu;

volatile sig_atomic_t flag = TRUE;
uint8_t fifoBuffer[64];
uint16_t packetSize;
uint8_t devStatus;
VectorFloat gravity;
float ypr[3];
Quaternion q;

void sig_handler(int signum)
{
	flag = FALSE;
}

int main(void) 
{
	mraa_result_t status = MRAA_SUCCESS;
    	struct sigaction action;
	uint8_t ret;
	float degree;
	char imu_data[8];
	uint16_t fifoCount;
	
    	int baudrate = 9600, stopbits = 1, databits = 8;
        mraa_uart_parity_t parity = MRAA_UART_PARITY_NONE;
    	unsigned int ctsrts = FALSE, xonxoff = FALSE;
        const char *name = NULL, *dev_path = "/dev/ttyACM0";

    	memset(&action, 0, sizeof(struct sigaction));
    	action.sa_handler = sig_handler;
    	sigaction(SIGINT, &action, NULL);


	uart = mraa_uart_init_raw(dev_path);
	if (uart == NULL)
		return -1;

	status = mraa_uart_settings(-1, &dev_path, &name, &baudrate, &databits, 
			&stopbits, &parity, &ctsrts, &xonxoff);
        if (status != MRAA_SUCCESS)
		return -1;

	mpu.initialize();
	ret = mpu.dmpInitialize();
	if (ret != 0) {
		printf("Failed to Initialize DMP\n");
		return -1;
	}

	printf("Enabling DMP...\n");
	mpu.setDMPEnabled(true);
	packetSize = mpu.dmpGetFIFOPacketSize();
	usleep(200000);
	while(flag) {
		fifoCount = mpu.getFIFOCount();
		if (fifoCount == 1024) {
			/* reset FIFO during overflow */
			mpu.resetFIFO();
			printf("FIFO overflow!\n");
		} else if (fifoCount >= 42) {
			/* read FIFO */
			mpu.getFIFOBytes(fifoBuffer, packetSize);

			/* get euler angles in degrees */
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			degree = ypr[2] * 180/M_PI;

			memset(imu_data, 0, sizeof(imu_data));
			
			/* send imu data to arduino */
			sprintf(imu_data, "%f", degree);
			strcpy(&imu_data[7], "\n");
			printf("%s", imu_data);
			mraa_uart_write(uart, (const char *)imu_data, sizeof(imu_data));
		
			}
		usleep(TIME_DELAY);
	}

	mraa_uart_stop(uart);

	return 0;
}
