#ifndef M3508_MOTOR_H
#define M3508_MOTOR_H

#include "main.h"
#include "pid_controller.h"
#include "can_communication.h"
#include <math.h>

#define MOTOR_COUNT 4
#define GEAR_RATIO 19.0f    // M3508减速比
#define ENCODER_RESOLUTION 8192.0f  // 编码器分辨率

typedef struct {
    uint8_t id;                    // 电机ID
    float target_angle;            // 目标角度 (度)
    float target_speed;            // 目标速度 (RPM)
    float current_angle;           // 当前角度 (度)
    float current_speed;           // 当前速度 (RPM)
    float total_angle;             // 累计角度 (度)

    PID_Controller_t speed_pid;    // 速度环PID
    PID_Controller_t position_pid; // 位置环PID

    int16_t output_current;        // 输出电流
    uint16_t last_encoder;         // 上次编码器值
    int32_t encoder_rounds;        // 编码器圈数
} M3508_Motor_t;

extern M3508_Motor_t motors[MOTOR_COUNT];

void M3508_Init(void);
void M3508_UpdateFeedback(uint8_t motor_id);
void M3508_SetTargetAngle(uint8_t motor_id, float target_angle);
void M3508_SetTargetSpeed(uint8_t motor_id, float target_speed);
void M3508_ControlUpdate(void);

#endif