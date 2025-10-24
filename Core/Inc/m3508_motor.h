#ifndef M3508_MOTOR_H
#define M3508_MOTOR_H

#include "main.h"
#include "pid_controller.h"
#include "can_communication.h"
#include <math.h>

#define GEAR_RATIO 19.0f
#define ENCODER_RESOLUTION 8192.0f

typedef enum {
    CONTROL_SPEED = 0,
    CONTROL_POSITION = 1,
    CONTROL_CASCADE = 2
} Control_Type_t;

typedef enum {
    CONTROL_SOURCE_BUTTON = 0,
    CONTROL_SOURCE_VOFA = 1
} Control_Source_t;

typedef struct {
    // 目标值
    float target_speed;
    float target_position;

    // 当前值
    float current_speed;
    float current_position;

    // PID控制器
    PID_Controller_t speed_pid;
    PID_Controller_t position_pid;

    // 控制参数
    int16_t output_current;
    uint16_t last_encoder;
    int32_t encoder_rounds;

    // 状态标志
    uint8_t initialized;
    uint8_t enabled;
    uint8_t direction;
    uint8_t control_type;
    uint8_t control_source;
    uint8_t last_control_type;
} M3508_Motor_t;

extern M3508_Motor_t motor;

void M3508_Init(void);
void M3508_UpdateFeedback(void);
void M3508_SetTargetSpeed(float target_speed);
void M3508_SetTargetPosition(float target_position);
void M3508_SetCascadeTarget(float position, float speed);
void M3508_SetSpeedPID(float kp, float ki, float kd);
void M3508_SetPositionPID(float kp, float ki, float kd);
void M3508_ControlUpdate(void);

#endif