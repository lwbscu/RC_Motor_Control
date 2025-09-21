#include "pid_controller.h"

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float max_output, float max_integral) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->max_integral = max_integral;

    pid->target = 0;
    pid->current = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
}

float PID_Calculate(PID_Controller_t *pid, float target, float current) {
    pid->target = target;
    pid->current = current;
    pid->error = target - current;

    // 积分计算
    pid->integral += pid->error;
    // 积分限幅
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }

    // 微分计算
    pid->derivative = pid->error - pid->last_error;

    // PID输出
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;

    // 输出限幅
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < -pid->max_output) {
        pid->output = -pid->max_output;
    }

    pid->last_error = pid->error;

    return pid->output;
}

void PID_Reset(PID_Controller_t *pid) {
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
}