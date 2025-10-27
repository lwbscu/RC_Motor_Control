#ifndef M3508_MOTOR_H
#define M3508_MOTOR_H

#include "main.h"
#include "pid_controller.h"
#include "can_communication.h"
#include <math.h>

// 使用手册中的精确减速比
#define GEAR_RATIO (3591.0f / 187.0f)
// 编码器分辨率 0-8191
#define ENCODER_RESOLUTION 8192.0f

typedef enum {
    CONTROL_SPEED = 0,    // 模式保留，但在实现中将不执行任何操作
    CONTROL_POSITION = 1, // 唯一有效的控制模式
    CONTROL_CASCADE = 2   // 模式保留，但在实现中将不执行任何操作
} Control_Type_t;

typedef enum {
    CONTROL_SOURCE_BUTTON = 0,
    CONTROL_SOURCE_VOFA = 1
} Control_Source_t;

typedef struct {
    // 目标值
    float target_position;

    // 当前值
    float current_position; // 单位：减速箱输出轴圈数

    // PID控制器 (仅位置环)
    PID_Controller_t position_pid;

    // 控制参数
    int16_t output_current;
    uint16_t last_encoder;
    int32_t encoder_rounds; // 电机转子累计圈数

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
void M3508_SetTargetPosition(float target_position);
void M3508_SetPositionPID(float kp, float ki, float kd);
void M3508_ControlUpdate(void);

#endif