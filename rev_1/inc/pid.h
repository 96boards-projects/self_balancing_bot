/*
 * Title: PID controller
 * Author: Manivannan Sadhasivam
 * Copyright (c) 2017 Linaro Ltd.
 * All rights reserved.
 */

#ifndef PID_H
#define PID_H

struct pid_ctrl {
	float last_err;
	float cum_err;
	float kp, ki, kd;
};

/**
 * Initialize PID controller
 * 
 * @param _kp Proportional term
 * @param _ki Integral term
 * @param _kd Derivative term
 * @param pid PID struct
 */
void pid_init(float _kp, float _ki, float _kd, struct pid_ctrl *pid);

/**
 * Calculate step value based on PID controller
 * 
 * @param val Current value
 * @param set_point Desired value
 * @param pid PID struct
 * @return PID output
 */
float pid_step(float val, float set_point, float dt, struct pid_ctrl *pid);

#endif
