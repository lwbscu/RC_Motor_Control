#include "m3508_motor.h"

M3508_Motor_t motor;

void M3508_Init(void) {
    motor.target_speed = 0;
    motor.current_speed = 0;
    motor.output_current = 0;
    motor.last_encoder = 0;
    motor.encoder_rounds = 0;
    motor.initialized = 0;

    // 只初始化速度环PID - 纯P控制器
    PID_Init(&motor.speed_pid, 10.0f, 0.0f, 0.0f, 500.0f, 10000.0f);
}

void M3508_UpdateFeedback(void) {
    if (!motor_data.data_updated) return;

    motor_data.data_updated = 0;
    motor.current_speed = motor_data.speed;

    uint16_t current_encoder = motor_data.angle;

    if (!motor.initialized) {
        motor.last_encoder = current_encoder;
        motor.initialized = 1;
        return;
    }

    int16_t delta_encoder = current_encoder - motor.last_encoder;

    if (delta_encoder > 4096) {
        delta_encoder -= 8192;
        motor.encoder_rounds--;
    } else if (delta_encoder < -4096) {
        delta_encoder += 8192;
        motor.encoder_rounds++;
    }

    motor.last_encoder = current_encoder;
}

void M3508_SetTargetSpeed(float target_speed) {
    motor.target_speed = target_speed;
}

void M3508_ControlUpdate(void) {
    if (!motor.initialized) return;

    // 只使用速度环控制
    motor.output_current = (int16_t)PID_Calculate(&motor.speed_pid, motor.target_speed, motor.current_speed);

    CAN_SendMotorCommand(motor.output_current);
}