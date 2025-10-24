#include "m2006_motor.h"

M2006_Motor_t motor;

// =================================================================
// 关键修改：控制周期 (TIM2 中断 1ms)
#define CONTROL_PERIOD_S 0.001f
// =================================================================


void M2006_Init(void) {
    motor.target_position_turns = 0;
    motor.current_position_turns = 0;
    motor.target_position_radians = 0;
    motor.current_position_radians = 0;
    motor.current_speed_rad_s = 0;
    motor.output_current = 0;
    motor.last_encoder = 0;      // 在Init中也初始化
    motor.total_encoder_ticks = 0; // 初始化累积ticks
    motor.initialized = 0;
    motor.enabled = 0; // 默认禁用
    motor.direction = 0;

    // =================================================================
    // 关键修改：提供新的PD参数起始值
    // 因为反馈Bug已修复，原来的参数 (30, 20) 可能过于激进
    // 我们从一个更小的值开始调
    // =================================================================
    PID_Init(&motor.position_pid,
             50.0f,     // Kp (新的起始值)
             0.0f,      // Ki (保持0)
             1.0f,      // Kd (新的起始值)
             10000.0f,  // MaxOutput (C610电流极限)
             0.0f);     // MaxIntegral (不使用积分)
}


// =================================================================
// M2006_UpdateFeedback() - (Bug修复后，此函数逻辑正确，无需修改)
// =================================================================
void M2006_UpdateFeedback(void) {
    if (!motor_data.data_updated) return;
    motor_data.data_updated = 0;

    // 1. 更新内部速度 (单位: rad/s)
    motor.current_speed_rad_s = (motor_data.speed / 60.0f) * (2.0f * M_PI);

    // 2. 更新内部位置 (单位: rad)
    uint16_t current_encoder = motor_data.angle; // 0 - 8191

    if (!motor.initialized) {
        // =================================================================
        // 关键修正：
        // 第一次收到数据时，我们强制定义当前位置为 0 点
        // 并且同步目标点也为 0，并重置PID
        // =================================================================
        motor.last_encoder = current_encoder;       // 1. 记录当前编码器值
        motor.total_encoder_ticks = 0;            // 2. 强制总ticks为 0
        motor.current_position_radians = 0;       // 3. 强制当前弧度为 0
        motor.current_position_turns = 0;         // 4. 强制当前圈数为 0

        motor.target_position_radians = 0;        // 5. 同步目标弧度为 0
        motor.target_position_turns = 0;          // 6. 同步目标圈数为 0

        PID_Reset(&motor.position_pid);           // 7. 重置PID积分器

        motor.initialized = 1;                    // 8. 完成初始化

    } else {
        // --- 修正后的累积 ticks 逻辑 ---
        // (此逻辑在 1kHz 频率下是正确的)
        int16_t delta_encoder = current_encoder - motor.last_encoder;

        // 处理 rollover
        if (delta_encoder > 4096) { // 发生下溢 (e.g., 100 -> 8000), 实际是后退
            delta_encoder -= 8192; // 得到负的变化量
        } else if (delta_encoder < -4096) { // 发生上溢 (e.g., 8000 -> 100), 实际是前进
            delta_encoder += 8192; // 得到正的变化量
        }

        motor.total_encoder_ticks += delta_encoder; // 累加总ticks
        motor.last_encoder = current_encoder;
    }

    // 计算内部当前位置 (弧度)
    motor.current_position_radians = motor.total_encoder_ticks * RAD_PER_ENCODER_TICK;

    // 计算外部显示位置 (输出轴圈数)
    motor.current_position_turns = motor.current_position_radians / (GEAR_RATIO * 2.0f * M_PI);
}

// =================================================================
// M2006_SetTargetPosition() - (无需修改)
// =================================================================
void M2006_SetTargetPosition(float target_position_turns) {
    motor.target_position_turns = target_position_turns; // 保存外部目标（可选）

    // 核心转换：输出轴圈数 -> 电机轴弧度
    motor.target_position_radians = target_position_turns * GEAR_RATIO * (2.0f * M_PI);
}

void M2006_SetPositionPID(float kp, float ki, float kd) {
    motor.position_pid.kp = kp;
    // motor.position_pid.ki = ki; // Ki 已被禁用
    motor.position_pid.kd = kd;
}

// =================================================================
// 关键修改：M2006_ControlUpdate() - 实现纯 PD 控制
// =================================================================
void M2006_ControlUpdate(void) {
    if (!motor.initialized) {
        CAN_SendMotorCommand(0);
        return;
    }

    if (!motor.enabled) {
        motor.output_current = 0;
        PID_Reset(&motor.position_pid);
        CAN_SendMotorCommand(motor.output_current);
        return;
    }

    PID_Controller_t *pid = &motor.position_pid;

    // PID 计算现在完全基于弧度和弧度/秒
    pid->target = motor.target_position_radians;   // 内部目标 (rad)
    pid->current = motor.current_position_radians; // 内部当前位置 (rad)
    pid->error = pid->target - pid->current;       // 误差 (rad)

    // =================================================================
    // 关键修正：移除积分项 (I)
    // =================================================================
    // pid->integral += pid->error * CONTROL_PERIOD_S; // 移除I项
    // if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    // else if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    float p_term = pid->kp * pid->error;        // P项
    // float i_term = pid->ki * pid->integral;     // 移除I项

    // D项 (基于测量值，即电机速度)
    // 注意：d_term 是用来产生“阻尼”的，所以是从输出中“减去”
    float d_term = pid->kd * motor.current_speed_rad_s;

    // PID 输出 = P - D (PD 控制器)
    pid->output = p_term - d_term;

    // 方向控制：只反转最终输出
    if (motor.direction) {
        pid->output = -pid->output;
    }

    // 输出限幅 (电流)
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    else if (pid->output < -pid->max_output) pid->output = -pid->max_output;

    motor.output_current = (int16_t)pid->output;
    CAN_SendMotorCommand(motor.output_current);
}