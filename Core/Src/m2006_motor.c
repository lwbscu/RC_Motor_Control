#include "m2006_motor.h"

// 关键修改：定义 motor 数组
M2006_Motor_t motor[2];

// =================================================================
// 控制周期 (TIM2 中断 1ms)
#define CONTROL_PERIOD_S 0.001f
// =================================================================


void M2006_Init(void) {
    // 循环初始化两个电机
    for (int i = 0; i < 2; i++) {
        motor[i].target_position_turns = 0;
        motor[i].current_position_turns = 0;
        motor[i].target_position_radians = 0;
        motor[i].current_position_radians = 0;
        motor[i].current_speed_rad_s = 0;
        motor[i].output_current = 0;
        motor[i].last_encoder = 0;
        motor[i].total_encoder_ticks = 0;
        motor[i].initialized = 0;
        motor[i].enabled = 0; // 默认禁用
        // motor[i].direction 稍后单独设置

        // 两个电机使用相同的PID参数
        PID_Init(&motor[i].position_pid,
                 111.0f,     // Kp (起始值)
                 0.0f,      // Ki (保持0)
                 10.0f,      // Kd (起始值)
                 10000.0f,  // MaxOutput (C610电流极限)
                 0.0f);     // MaxIntegral (不使用积分)
    }

    // =================================================================
    // 关键修改：设置方向标志 (用于 SetTargetPosition)
    // =================================================================
    motor[0].direction = 1; // 电机1 (ID 3) 标记为反向
    motor[1].direction = 0; // 电机2 (ID 4) 标记为正向
}


void M2006_UpdateFeedback(void) {
    // 此函数逻辑不变，循环更新两个电机的反馈
    for (int i = 0; i < 2; i++) {
        if (!motor_data[i].data_updated) continue;
        motor_data[i].data_updated = 0;

        motor[i].current_speed_rad_s = (motor_data[i].speed / 60.0f) * (2.0f * M_PI);
        uint16_t current_encoder = motor_data[i].angle;

        if (!motor[i].initialized) {
            motor[i].last_encoder = current_encoder;
            motor[i].total_encoder_ticks = 0;
            motor[i].current_position_radians = 0;
            motor[i].current_position_turns = 0;
            motor[i].target_position_radians = 0;
            motor[i].target_position_turns = 0;
            PID_Reset(&motor[i].position_pid);
            motor[i].initialized = 1;
        } else {
            int16_t delta_encoder = current_encoder - motor[i].last_encoder;

            if (delta_encoder > 4096) {
                delta_encoder -= 8192;
            } else if (delta_encoder < -4096) {
                delta_encoder += 8192;
            }

            motor[i].total_encoder_ticks += delta_encoder;
            motor[i].last_encoder = current_encoder;
        }

        motor[i].current_position_radians = motor[i].total_encoder_ticks * RAD_PER_ENCODER_TICK;
        motor[i].current_position_turns = motor[i].current_position_radians / (GEAR_RATIO * 2.0f * M_PI);
    }
}

// =================================================================
// 关键修改：在设置目标时应用反转
// =================================================================
void M2006_SetTargetPosition(float target_position_turns) {
    for (int i = 0; i < 2; i++) {

        float effective_target_turns = target_position_turns;

        // 核心逻辑：如果电机被标记为反向，则将其目标位置取反
        if (motor[i].direction == 1) {
            effective_target_turns = -target_position_turns;
        }

        motor[i].target_position_turns = effective_target_turns; // 保存反转后的外部目标

        // 核心转换：输出轴圈数 -> 电机轴弧度
        motor[i].target_position_radians = effective_target_turns * GEAR_RATIO * (2.0f * M_PI);
    }
}

// 设置PID时，同时设置两个电机 (此函数不变)
void M2006_SetPositionPID(float kp, float ki, float kd) {
    for (int i = 0; i < 2; i++) {
        motor[i].position_pid.kp = kp;
        // motor[i].position_pid.ki = ki; // Ki 已被禁用
        motor[i].position_pid.kd = kd;
    }
}

// =================================================================
// 关键修改：移除 ControlUpdate 末尾的输出反转
// =================================================================
void M2006_ControlUpdate(void) {
    int16_t output_currents[2] = {0, 0};

    // 循环计算两个电机的PID
    for (int i = 0; i < 2; i++) {
        if (!motor[i].initialized) {
            continue;
        }

        if (!motor[i].enabled) {
            motor[i].output_current = 0;
            PID_Reset(&motor[i].position_pid);
            continue;
        }

        PID_Controller_t *pid = &motor[i].position_pid;

        // PID 计算 (现在 target_position_radians 已经是正确的 +/- 值)
        pid->target = motor[i].target_position_radians;
        pid->current = motor[i].current_position_radians;
        pid->error = pid->target - pid->current;

        float p_term = pid->kp * pid->error;
        float d_term = pid->kd * motor[i].current_speed_rad_s;

        // PID 输出 = P - D (PD 控制器)
        pid->output = p_term - d_term;

        // 关键修改：移除此处的方向控制逻辑
        // if (motor[i].direction) { ... }

        // 输出限幅 (电流)
        if (pid->output > pid->max_output) pid->output = pid->max_output;
        else if (pid->output < -pid->max_output) pid->output = -pid->max_output;

        motor[i].output_current = (int16_t)pid->output;
        output_currents[i] = motor[i].output_current;
    }

    // 只调用一次CAN发送
    CAN_SendMotorCommand(output_currents[0], output_currents[1]);
}