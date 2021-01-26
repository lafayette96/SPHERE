#include "pid.h"

#define ERR_SUM_MAX		1000

struct pid_params
{
	float kp;
	float ki;
	float kd;
	float err;
	float err_sum;
	float err_last;
};

static struct pid_params pid_params;

void pid_init(float kp, float ki, float kd)
{
	pid_params.kp = kp;
	pid_params.ki = ki;
	pid_params.kd = kd;
	pid_params.err = 0;
	pid_params.err_sum = 0;
	pid_params.err_last = 0;
}

float pid_calculate(float set_val, float read_val)
{
	float err_d, u;

	pid_params.err = set_val - read_val;
	pid_params.err_sum += pid_params.err;

	if (pid_params.err_sum > ERR_SUM_MAX) {
		pid_params.err_sum = ERR_SUM_MAX;
	} else if (pid_params.err_sum < -ERR_SUM_MAX) {
		pid_params.err_sum = -ERR_SUM_MAX;
	}

	err_d = pid_params.err_last - pid_params.err;
	u = pid_params.kp * pid_params.err + pid_params.ki * pid_params.err_sum
			+ pid_params.kd * err_d;
	return u;
}
