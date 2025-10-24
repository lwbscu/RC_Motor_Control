#include "m2006_motor.h"

M2006_Motor_t motor;

// 修正后的Init函数
void M2006_Init(void) {
    motor.target_speed = 0; // 初始目标应为0
    motor.target_position = 0;
    motor.current_speed = 0;
    motor.current_position = 0;
    motor.output_current = 0;
    motor.last_encoder = 0;
    motor.encoder_rounds = 0;
    motor.initialized = 0;
    motor.enabled = 0; // 默认应为 0 (禁用)，等待VOFA+开启
    motor.direction = 0;
    motor.control_type = CONTROL_SPEED;
    motor.last_control_type = CONTROL_SPEED;
    motor.control_source = CONTROL_SOURCE_VOFA;

    // 1. 速度环PID (Kp, Ki, Kd, MaxOutput, MaxIntegral)
    PID_Init(&motor.speed_pid,
             15.0f,    // Kp (待整定)
             0.05f,    // Ki (待整定)
             0.0f,     // Kd (待整定)
             10000.0f, // MaxOutput (C610电流极限)
             5000.0f); // MaxIntegral (防饱和)

    // 2. 位置环PID (Kp, Ki, Kd, MaxOutput, MaxIntegral)
    PID_Init(&motor.position_pid,
             1.0f,     // Kp (待整定)
             0.0f,     // Ki (待整定, 可能为0)
             0.0f,     // Kd (待整定)
             5000.0f,     // MaxOutput (M2006最大转速约 8 rps)
             5000.0f);    // MaxIntegral (防饱和)
}


void M2006_UpdateFeedback(void) {
    if (!motor_data.data_updated) return;

    motor_data.data_updated = 0;
    motor.current_speed = motor_data.speed / 60.0f;

    uint16_t current_encoder = motor_data.angle;

    if (!motor.initialized) {
        motor.last_encoder = current_encoder;
        motor.initialized = 1;
        return;
    }

    if (motor.control_type == CONTROL_POSITION || motor.control_type == CONTROL_CASCADE) {
        int16_t delta_encoder = current_encoder - motor.last_encoder;

        if (delta_encoder > 4096) {
            delta_encoder -= 8192;
            motor.encoder_rounds--;
        } else if (delta_encoder < -4096) {
            delta_encoder += 8192;
            motor.encoder_rounds++;
        }

        motor.last_encoder = current_encoder;
        motor.current_position = (motor.encoder_rounds + (float)current_encoder / ENCODER_RESOLUTION) / GEAR_RATIO;
    } else {
        motor.current_position = 0;
        motor.encoder_rounds = 0;
        motor.last_encoder = current_encoder;
    }
}

void M2006_SetTargetSpeed(float target_speed) {
    motor.target_speed = target_speed;
}

void M2006_SetTargetPosition(float target_position) {
    motor.target_position = target_position;
}

void M2006_SetCascadeTarget(float position, float speed) {
    motor.target_position = position;
    motor.target_speed = speed;
}

void M2006_SetSpeedPID(float kp, float ki, float kd) {
    motor.speed_pid.kp = kp;
    motor.speed_pid.ki = ki;
    motor.speed_pid.kd = kd;
}

void M2006_SetPositionPID(float kp, float ki, float kd) {
    motor.position_pid.kp = kp;
    motor.position_pid.ki = ki;
    motor.position_pid.kd = kd;
}

// =================================================================
// 关键修改：M2006_ControlUpdate()
// =================================================================
void M2006_ControlUpdate(void) {
    if (!motor.initialized) {
        // 如果电机还未收到过CAN消息，保持发送0电流
        CAN_SendMotorCommand(0);
        return;
    }

    // --- 这是新的核心逻辑 ---
    // 检查电机是否被禁用
    if (!motor.enabled) {
        // 如果被禁用 (enable:0)
        motor.output_current = 0;     // 1. 强制设置输出电流为0
        PID_Reset(&motor.speed_pid);  // 2. 重置PID状态 (清除积分和last_error)
        PID_Reset(&motor.position_pid);

        // 3. 主动发送 0 电流指令，而不是让电调超时
        CAN_SendMotorCommand(motor.output_current);
        return; // 退出，不执行后续PID计算
    }
    // --- 新逻辑结束 ---

    // 如果程序运行到这里，说明 motor.enabled == 1

    // 检测控制模式切换并重置相应PID
    if (motor.control_type != motor.last_control_type) {
        switch (motor.control_type) {
            case CONTROL_SPEED:
                PID_Reset(&motor.position_pid);
                break;
            case CONTROL_POSITION:
                PID_Reset(&motor.speed_pid);
                PID_Reset(&motor.position_pid);
                motor.target_position = motor.current_position;
                break;
            case CONTROL_CASCADE:
                PID_Reset(&motor.speed_pid);
                PID_Reset(&motor.position_pid);
                break;
        }
        motor.last_control_type = motor.control_type;
    }

    float speed_target = motor.target_speed;

    switch (motor.control_type) {
        case CONTROL_SPEED:
            // 直接速度控制
            break;

        case CONTROL_POSITION:
            // 位置环控制
            // PID_Calculate 内部会使用 position_pid.max_output (即 8.0f) 进行限幅
            speed_target = PID_Calculate(&motor.position_pid,
                                         motor.target_position,
                                         motor.current_position);
            break;

        case CONTROL_CASCADE:
        {
            // 串级控制
            float position_output = PID_Calculate(&motor.position_pid,
                                                  motor.target_position,
                                                  motor.current_position);

            float speed_limit = fabs(motor.target_speed);
            if (speed_limit < 0.1f) speed_limit = 0.1f;

            if (position_output > speed_limit) {
                position_output = speed_limit;
            } else if (position_output < -speed_limit) {
                position_output = -speed_limit;
            }
            speed_target = position_output;
        }
            break;
    }

    // 方向控制
    if (motor.direction) speed_target = -speed_target;

    // 速度环PID计算 (核心)
    motor.output_current = (int16_t)PID_Calculate(&motor.speed_pid, speed_target, motor.current_speed);

    // 发送最终计算的电流值
    CAN_SendMotorCommand(motor.output_current);
}