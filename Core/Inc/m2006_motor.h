#ifndef M2006_MOTOR_H
#define M2006_MOTOR_H

#include "main.h"
#include "pid_controller.h"
#include "can_communication.h"
#include <math.h> // 需要包含 math.h 以使用 M_PI

// M_PI 定义 (如果 math.h 中没有)
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 物理参数 [cite: 241, 437]
#define GEAR_RATIO 36.0f
#define ENCODER_RESOLUTION 8192.0f
#define RAD_PER_ENCODER_TICK (2.0f * M_PI / ENCODER_RESOLUTION) // 每编码器计数对应的弧度

typedef struct {
    // --- 接口单位 (输出轴) ---
    float target_position_turns;   // 目标位置 (圈数) - VOFA输入/设置
    float current_position_turns;  // 当前位置 (圈数) - VOFA显示

    // --- 内部单位 (电机轴) ---
    float target_position_radians; // 内部PID目标位置 (弧度)
    float current_position_radians;// 内部PID当前位置 (弧度)
    float current_speed_rad_s;     // 内部PID当前速度 (弧度/秒)

    // PID控制器
    PID_Controller_t position_pid;

    // 控制参数
    int16_t output_current;
    uint16_t last_encoder;      // 上一次的原始编码器值 (0-8191)
    int64_t total_encoder_ticks; // 累积的总编码器tick数 (用于计算弧度)

    // 状态标志
    uint8_t initialized;
    uint8_t enabled;
    uint8_t direction; // 0: 正向, 1: 反向

} M2006_Motor_t;

// 关键修改：将 motor 扩展为数组, [0] 对应 ID 3, [1] 对应 ID 4
extern M2006_Motor_t motor[2];

// 函数声明保持不变，但内部实现会改变
void M2006_Init(void);
void M2006_UpdateFeedback(void);
void M2006_SetTargetPosition(float target_position_turns); // 参数名明确单位
void M2006_SetPositionPID(float kp, float ki, float kd);
void M2006_ControlUpdate(void);

#endif