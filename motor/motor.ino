// Title: Motor controller sketch for Self Balancing Bot
// Author: Manivannan Sadhasivam
// Copyright (c) 2017 Linaro Ltd.
// All rights reserved.
  
#include <PID_v1.h>
#include <LMotorController.h>

#define MIN_ABS_SPEED 20

// PID constants
double Kp = 50;   
double Kd = 2;
double Ki = 0;

int led = 13;

double originalSetpoint = 173;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.5;

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
    Serial.begin(9600);
    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
    pinMode(led, OUTPUT);
}
  
void loop() {
    String val = Serial.readStringUntil('\n');
    digitalWrite(led, HIGH);
    val.toCharArray(buffer, 10);
    if (buffer != NULL) {
      digitalWrite(led, HIGH);
        pid.Compute();
        motorController.move(output, MIN_ABS_SPEED);
    }
  }
