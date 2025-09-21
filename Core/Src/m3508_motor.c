#include "m3508_motor.h"

M3508_Motor_t motors[MOTOR_COUNT];

void M3508_Init(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i].id = i;
        motors[i].target_angle = 0;
        motors[i].target_speed = 0;
        motors[i].current_angle = 0;
        motors[i].current_speed = 0;
        motors[i].total_angle = 0;
        motors[i].output_current = 0;
        motors[i].last_encoder = 0;
        motors[i].encoder_rounds = 0;
        motors[i].initialized = 0;

        // 速度环PID - 加大参数
        PID_Init(&motors[i].speed_pid, 15.0f, 0.5f, 0.2f, 16000.0f, 8000.0f);

        // 位置环PID - 加大参数  
        PID_Init(&motors[i].position_pid, 2.0f, 0.0f, 0.3f, 1500.0f, 800.0f);
    }
}

void M3508_UpdateFeedback(uint8_t motor_id) {
    M3508_Motor_t *motor = &motors[motor_id];
    M3508_Motor_Data_t *data = &motor_data[motor_id];

    // 检查数据是否更新
    if (!data->data_updated) return;
    
    data->data_updated = 0;
    motor->current_speed = data->speed;

    uint16_t current_encoder = data->angle;
    
    // 首次初始化
    if (!motor->initialized) {
        motor->last_encoder = current_encoder;
        motor->initialized = 1;
        return;
    }

    int16_t delta_encoder = current_encoder - motor->last_encoder;

    // 处理编码器值溢出
    if (delta_encoder > 4096) {
        delta_encoder -= 8192;
        motor->encoder_rounds--;
    } else if (delta_encoder < -4096) {
        delta_encoder += 8192;
        motor->encoder_rounds++;
    }

    motor->last_encoder = current_encoder;

    // 计算总角度
    motor->total_angle = (motor->encoder_rounds * 8192.0f + current_encoder) / ENCODER_RESOLUTION * 360.0f / GEAR_RATIO;
    motor->current_angle = motor->total_angle;
}

void M3508_SetTargetAngle(uint8_t motor_id, float target_angle) {
    motors[motor_id].target_angle = target_angle;
}

void M3508_SetTargetSpeed(uint8_t motor_id, float target_speed) {
    motors[motor_id].target_speed = target_speed;
}

void M3508_ControlUpdate(void) {
    int16_t motor_currents[4] = {0};

    for (int i = 0; i < MOTOR_COUNT; i++) {
        M3508_Motor_t *motor = &motors[i];

        if (!motor->initialized) continue;

        if (motor->target_angle != 0) {
            // 位置环控制
            float speed_target = PID_Calculate(&motor->position_pid, motor->target_angle, motor->current_angle);
            motor->output_current = (int16_t)PID_Calculate(&motor->speed_pid, speed_target, motor->current_speed);
        } else if (motor->target_speed != 0) {
            // 速度环控制
            motor->output_current = (int16_t)PID_Calculate(&motor->speed_pid, motor->target_speed, motor->current_speed);
        } else {
            motor->output_current = 0;
        }

        motor_currents[i] = motor->output_current;
    }
    //CAN_SendMotorCommand(2000, 2000, 2000, 2000);

    CAN_SendMotorCommand(motor_currents[0], motor_currents[1], motor_currents[2], motor_currents[3]);
}