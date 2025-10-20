#include "m3508_motor.h"

M3508_Motor_t motor;

void M3508_Init(void) {
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
    motor.last_control_type = CONTROL_SPEED;  // 添加上一次控制模式记录
    motor.control_source = CONTROL_SOURCE_VOFA;

    // 初始化速度环PID
    PID_Init(&motor.speed_pid, 35.0f, 0.07f, 0.0f, 8000.0f, 4000.0f);

    // 初始化位置环PID
    PID_Init(&motor.position_pid, 57.0f, 0.0f, 251.0f, 150.0f, 4000.0f);
}

void M3508_UpdateFeedback(void) {
    if (!motor_data.data_updated) return;

    motor_data.data_updated = 0;
    motor.current_speed = motor_data.speed / 60.0f; // 转换为转/秒

    uint16_t current_encoder = motor_data.angle;

    if (!motor.initialized) {
        motor.last_encoder = current_encoder;
        motor.initialized = 1;
        return;
    }

    // 只在位置控制或串级控制模式下更新位置
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

        // 计算当前位置（圈数）
        motor.current_position = (motor.encoder_rounds + (float)current_encoder / ENCODER_RESOLUTION)/19;
    } else {
        // 速度环模式下，保持位置为0，重置位置相关变量
        motor.current_position = 0;
        motor.encoder_rounds = 0;
        motor.last_encoder = current_encoder;  // 更新编码器值但不累积位置
    }
}

void M3508_SetTargetSpeed(float target_speed) {
    motor.target_speed = target_speed;
}

void M3508_SetTargetPosition(float target_position) {
    motor.target_position = target_position;
}

void M3508_SetCascadeTarget(float position, float speed) {
    motor.target_position = position;
    motor.target_speed = speed;
}

void M3508_SetSpeedPID(float kp, float ki, float kd) {
    motor.speed_pid.kp = kp;
    motor.speed_pid.ki = ki;
    motor.speed_pid.kd = kd;
}

void M3508_SetPositionPID(float kp, float ki, float kd) {
    motor.position_pid.kp = kp;
    motor.position_pid.ki = ki;
    motor.position_pid.kd = kd;
}

// 修复M3508_ControlUpdate()中的串级控制部分
void M3508_ControlUpdate(void) {
    if (!motor.initialized || !motor.enabled) return;

    // 检测控制模式切换并重置相应PID
    if (motor.control_type != motor.last_control_type) {
        switch (motor.control_type) {
            case CONTROL_SPEED:
                // 切换到速度模式时，重置位置环PID，避免积分累积
                PID_Reset(&motor.position_pid);
                break;

            case CONTROL_POSITION:
                // 切换到位置模式时，重置速度环和位置环PID
                PID_Reset(&motor.speed_pid);
                PID_Reset(&motor.position_pid);
                // 将当前位置设为目标位置，避免突跳
                motor.target_position = motor.current_position;
                break;

            case CONTROL_CASCADE:
                // 切换到串级模式时，重置所有PID
                PID_Reset(&motor.speed_pid);
                PID_Reset(&motor.position_pid);
                // 注意：不要重置目标位置，保持用户设定值
                break;
        }
        motor.last_control_type = motor.control_type;
    }

    float speed_target = motor.target_speed;

    switch (motor.control_type) {
        case CONTROL_SPEED:
            // 直接速度控制，不调用位置环PID
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
            // 串级控制：位置环输出受target_speed限制
            float position_output = PID_Calculate(&motor.position_pid,
                                                  motor.target_position,
                                                  motor.current_position);

            // 使用target_speed作为速度限制
            float speed_limit = fabs(motor.target_speed);
            if (speed_limit < 0.1f) speed_limit = 0.1f; // 最小速度限制，避免除零

            // 将位置环输出限制在用户设定的速度范围内
            if (position_output > speed_limit) {
                position_output = speed_limit;
            } else if (position_output < -speed_limit) {
                position_output = -speed_limit;
            }

            // 位置环输出作为速度环目标
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
