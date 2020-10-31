#ifndef PID_H_
#define PID_H_

#include "platform_specific.h"

void pid_init(float kp, float ki, float kd);

float pid_calculate(float set_val, float read_val);

#endif /* PID_H_ */
