#include "m2006_motor.h"

// 重命名全局变量
M2006_Motor_t motor;

// 重命名函数
void M2006_Init(void) {
    motor.target_speed = 50;
    motor.target_position = 0;
    motor.current_speed = 0;
    motor.current_position = 0;
    motor.output_current = 0;
    motor.last_encoder = 0;
    motor.encoder_rounds = 0;
    motor.initialized = 0;
    motor.enabled = 1;
    motor.direction = 0;
    motor.control_type = CONTROL_SPEED;
    motor.last_control_type = CONTROL_SPEED;
    motor.control_source = CONTROL_SOURCE_VOFA;

    // 关键修改：更新PID参数以适应M2006和C610
    // C610 电流范围: -10000 ~ +10000
    // !!! 警告：以下PID参数未经整定，仅供启动测试，必须使用VOFA+重新整定 !!!
    // 速度环PID (Kp, Ki, Kd, MaxOutput, MaxIntegral)
    PID_Init(&motor.speed_pid, 15.0f, 0.05f, 0.0f, 10000.0f, 5000.0f);

    // 位置环PID (Kp, Ki, Kd, MaxOutput (速度限制), MaxIntegral)
    PID_Init(&motor.position_pid, 30.0f, 0.0f, 100.0f, 150.0f, 4000.0f);
}

// 重命名函数
void M2006_UpdateFeedback(void) {
    if (!motor_data.data_updated) return;

    motor_data.data_updated = 0;
    // C610 返回 RPM，转换为 转/秒 (RPS)，此逻辑不变
    motor.current_speed = motor_data.speed / 60.0f;

    uint16_t current_encoder = motor_data.angle;

    if (!motor.initialized) {
        motor.last_encoder = current_encoder;
        motor.initialized = 1;
        return;
    }

    // 只在位置控制或串级控制模式下更新位置
    if (motor.control_type == CONTROL_POSITION || motor.control_type == CONTROL_CASCADE) {
        int16_t delta_encoder = current_encoder - motor.last_encoder;

        // 编码器圈数累积逻辑 (8192 / 2 = 4096)
        if (delta_encoder > 4096) {
            delta_encoder -= 8192;
            motor.encoder_rounds--;
        } else if (delta_encoder < -4096) {
            delta_encoder += 8192;
            motor.encoder_rounds++;
        }

        motor.last_encoder = current_encoder;

        // 关键修改：计算当前位置（圈数），使用新的 GEAR_RATIO
        motor.current_position = (motor.encoder_rounds + (float)current_encoder / ENCODER_RESOLUTION) / GEAR_RATIO;
    } else {
        // 速度环模式下，重置位置相关变量
        motor.current_position = 0;
        motor.encoder_rounds = 0;
        motor.last_encoder = current_encoder;
    }
}

// 重命名函数
void M2006_SetTargetSpeed(float target_speed) {
    motor.target_speed = target_speed;
}

// 重命名函数
void M2006_SetTargetPosition(float target_position) {
    motor.target_position = target_position;
}

// 重命名函数
void M2006_SetCascadeTarget(float position, float speed) {
    motor.target_position = position;
    motor.target_speed = speed;
}

// 重命名函数
void M2006_SetSpeedPID(float kp, float ki, float kd) {
    motor.speed_pid.kp = kp;
    motor.speed_pid.ki = ki;
    motor.speed_pid.kd = kd;
}

// 重命名函数
void M2006_SetPositionPID(float kp, float ki, float kd) {
    motor.position_pid.kp = kp;
    motor.position_pid.ki = ki;
    motor.position_pid.kd = kd;
}

// 重命名函数
void M2006_ControlUpdate(void) {
    if (!motor.initialized || !motor.enabled) return;

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
            speed_target = PID_Calculate(&motor.position_pid,
                                         motor.target_position,
                                         motor.current_position);
            // 限制速度输出
            if (speed_target > 50.0f) speed_target = 50.0f;
            if (speed_target < -50.0f) speed_target = -50.0f;
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

    // 速度环PID计算
    motor.output_current = (int16_t)PID_Calculate(&motor.speed_pid, speed_target, motor.current_speed);

    CAN_SendMotorCommand(motor.output_current);
}