/*
 * Title: PID controller Implementation
 * Author: Manivannan Sadhasivam
 * Copyright (c) 2017 Linaro Ltd.
 * All rights reserved.
 */

#include "pid.h"

void pid_init(float _kp, float _ki, float _kd, struct pid_ctrl *pid)
{
        pid->last_err = 0;
        pid->cum_err = 0;
        pid->kp = _kp;
        pid->ki = _ki;
        pid->kd = _kd;
}

float pid_step(float val, float set_point, float dt, struct pid_ctrl *pid)
{
	float err;
	float _kp, _ki, _kd;
	
	/* calculate error value based on current value and set point*/
	err = val - set_point;

	/* calculate Kp */
	_kp = pid->kp * err;	

	/* calculate Ki */
	_ki = pid->ki * pid->cum_err * dt;
	pid->cum_err += err;

	/* calculate Kd */
	_kd = (pid->kd * (err - pid->last_err)) / dt;
	pid->last_err = err;

	/* pid algorithm */
	return (_kp + _ki - _kd);
}
