#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数

    float target;       // 目标值
    float current;      // 当前值
    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 积分累积
    float derivative;   // 微分

    float output;       // PID输出
    float max_output;   // 输出限幅
    float max_integral; // 积分限幅
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float max_output, float max_integral);
float PID_Calculate(PID_Controller_t *pid, float target, float current);
void PID_Reset(PID_Controller_t *pid);

#endif