#ifndef M2006_MOTOR_H
#define M2006_MOTOR_H

#include "main.h"
#include "pid_controller.h"
#include "can_communication.h"
#include <math.h>

// 关键修改：M2006 P36 减速比为 36:1
#define GEAR_RATIO 36.0f
// 编码器分辨率 (由C610电调提供，0-8191)
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

// 重命名结构体
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
} M2006_Motor_t;

// 重命名全局变量
extern M2006_Motor_t motor;

// 重命名函数
void M2006_Init(void);
void M2006_UpdateFeedback(void);
void M2006_SetTargetSpeed(float target_speed);
void M2006_SetTargetPosition(float target_position);
void M2006_SetCascadeTarget(float position, float speed);
void M2006_SetSpeedPID(float kp, float ki, float kd);
void M2006_SetPositionPID(float kp, float ki, float kd);
void M2006_ControlUpdate(void);

#endif