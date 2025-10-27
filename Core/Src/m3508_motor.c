#include "m3508_motor.h"

M3508_Motor_t motor;

void M3508_Init(void) {
    motor.target_position = 0;
    motor.current_position = 0;
    motor.output_current = 0;
    motor.last_encoder = 0;
    motor.encoder_rounds = 0;
    motor.initialized = 0;
    motor.enabled = 1;
    motor.direction = 0;
    motor.control_type = CONTROL_POSITION; // 默认进入位置模式
    motor.last_control_type = CONTROL_POSITION;
    motor.control_source = CONTROL_SOURCE_VOFA;

    // 初始化位置环PID
    // 注意：原K_p, K_i, K_d参数是为P->S串级环设计的，
    // 现在改为单P环，参数需要您在VOFA+上重新整定。
    // K_i 暂时设为0，K_p和K_d保留原值作为起点。
    // max_output 根据C620手册设为16384
    // max_integral 设为10000.0f
    PID_Init(&motor.position_pid, 100.0f, 0.0f, 0.0f, 6384.0f, 10000.0f);
}

void M3508_UpdateFeedback(void) {
    if (!motor_data.data_updated) return;

    motor_data.data_updated = 0;
    uint16_t current_encoder = motor_data.angle;

    if (!motor.initialized) {
        motor.last_encoder = current_encoder;
        motor.initialized = 1;
        return;
    }

    // --- 位置计算 ---
    // 1. 计算编码器增量并处理溢出 (8192 / 2 = 4096)
    int16_t delta_encoder = current_encoder - motor.last_encoder;
    if (delta_encoder > 4096) {
        delta_encoder -= 8192;
        motor.encoder_rounds--; // 反转
    } else if (delta_encoder < -4096) {
        delta_encoder += 8192;
        motor.encoder_rounds++; // 正转
    }
    motor.last_encoder = current_encoder;

    // 2. 计算电机转子的总圈数（带小数）
    float motor_shaft_turns = motor.encoder_rounds + (float)current_encoder / ENCODER_RESOLUTION;

    // 3. 计算减速箱输出轴的总圈数（您需要的目标）
    motor.current_position = motor_shaft_turns / GEAR_RATIO;
}

void M3508_SetTargetPosition(float target_position) {
    motor.target_position = target_position;
}


void M3508_SetPositionPID(float kp, float ki, float kd) {
    motor.position_pid.kp = kp;
    motor.position_pid.ki = ki;
    motor.position_pid.kd = kd;
}

void M3508_ControlUpdate(void) {
    if (!motor.initialized) {
        CAN_SendMotorCommand(0); // 未初始化则不发送电流
        return;
    }

    if (!motor.enabled) {
        CAN_SendMotorCommand(0); // 未使能则发送0电流
        PID_Reset(&motor.position_pid); // 重置PID
        return;
    }

    // 检测控制模式切换
    if (motor.control_type != motor.last_control_type) {
        PID_Reset(&motor.position_pid); // 切换模式时重置PID

        // 如果切换到位置模式，为防止电机突跳，将目标设为当前位置
        if (motor.control_type == CONTROL_POSITION) {
            motor.target_position = motor.current_position;
        }
        motor.last_control_type = motor.control_type;
    }

    // --- 单环位置PID控制 ---
    // 仅在位置模式下执行PID计算
    if (motor.control_type == CONTROL_POSITION) {
        // PID计算：输入为目标圈数和当前圈数，输出为电流值
        motor.output_current = (int16_t)PID_Calculate(&motor.position_pid,
                                                      motor.target_position,
                                                      motor.current_position);
    } else {
        // 如果处于其他模式（如SPEED或CASCADE），则输出0电流
        // 因为我们只实现位置环
        motor.output_current = 0;
    }

    // 方向控制
    if (motor.direction) {
        motor.output_current = -motor.output_current;
    }

    // 发送最终电流指令到CAN总线
    CAN_SendMotorCommand(motor.output_current);
}