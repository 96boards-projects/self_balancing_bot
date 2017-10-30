// Title: Motor controller sketch for Self Balancing Bot
// Author: Manivannan Sadhasivam
// Copyright (c) 2017 Linaro Ltd.
// All rights reserved.
  
#include <PID_v1.h>
#include <LMotorController.h>

#define MIN_ABS_SPEED 20

// PID constants
double Kp = 15;   
double Ki = 0;
double Kd = 0.2;

double setpoint = -2.4;
double input, output;

// initialize PID
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.9;

// connect motor controller pins to Arduino digital pins
// motor one
int ENA = 10;
int IN1 = 9;
int IN2 = 8;
// motor two
int ENB = 5;
int IN3 = 7;
int IN4 = 6;
  
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);
  
char buffer[10];
  
void setup()
{
    // initialize uart
    Serial.begin(9600);
    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
}
  
void loop() {
  if (Serial.available() > 0) {
    // read IMU data from 96Boards CE
    String val = Serial.readStringUntil('\n');
    val.toCharArray(buffer, 10);
    input = atof(buffer);
    // compute PID value for the IMU data
    pid.Compute();
    // drive motors according to the PID value
    motorController.move(output, MIN_ABS_SPEED);
  }
}
